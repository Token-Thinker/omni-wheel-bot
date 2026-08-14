#![no_std]
#![no_main]
#![recursion_limit = "512"]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(feature = "ble")]
mod ble;
mod board;
mod camera;
mod profile;
mod ui_assets;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::AnyPin,
    i2c::master::{Config as I2cConfig, I2c},
    rtc_cntl::{Rtc, Rwdt, RwdtStage},
    usb_serial_jtag::UsbSerialJtag,
    Blocking,
};
#[cfg(target_arch = "xtensa")]
use esp_hal::{interrupt::software::SoftwareInterrupt, system::Stack as CpuStack};
use esp_radio::{
    self,
    wifi::{
        ap::AccessPointConfig,
        sta::StationConfig,
        AuthenticationMethod,
        Config as WifiModeConfig,
        ControllerConfig,
        Interface,
        WifiController,
    },
};
use log::LevelFilter;
use owb_core::{
    ext::prelude::*,
    platform::{BoardRuntimeKind, CpuTopology},
};

type Bus<'d> = I2c<'d, Blocking>;
#[cfg(target_arch = "xtensa")]
const CONTROL_CORE_STACK_BYTES: usize = 16 * 1024;
// 12 sockets: 4 pwa workers + spare for embassy-net internals and TIME_WAIT
const S3_NET_SOCKETS: usize = 12;

// ── Heap layout ─────────────────────────────────────────────────────────────
//
// The ESP32-S3 has two accessible SRAM banks for heap:
//
//  • DRAM1 (dram_seg)  0x3FC88000 – 0x3FCDB700  334 KB
//    Static sections (non-heap) occupy ~178 KB (WiFi, embassy, StackResources,
//    etc.).  The CPU0 main-task stack lives in whatever DRAM1 is left after
//    statics: stack = 334 KB - 178 KB - HEAP_BYTES.
//    With HEAP_BYTES = 100 KB:  stack = 334 – 178 – 100 = 56 KB  ← enough for
//    the 14 KB Box::pin(run_with_router(…)) temporary on first task poll.
//
//  • DRAM2 (dram2_seg) 0x3FCDB700 – 0x3FCED710  72 KB
//    Reserved by memory.x for application heap use.  We add 70 KB here,
//    leaving ~2 KB as a safety margin below the 72 KB segment limit.
//
//  Total heap = 100 KB + 70 KB = 170 KB.
//  Runtime peak (measured): ~47 KB pre-existing + 4 × ~27 KB workers ≈ 157 KB.
//  Margin ≈ 13 KB (covers linked_list_allocator fragmentation overhead).

const DRAM2_HEAP_BYTES: usize = 70 * 1024;

#[link_section = ".dram2_uninit"]
static mut DRAM2_HEAP_MEM: core::mem::MaybeUninit<[u8; DRAM2_HEAP_BYTES]> =
    core::mem::MaybeUninit::uninit();

#[cfg(feature = "real-executors")]
const HEAP_BYTES: usize = 100 * 1024;
#[cfg(not(feature = "real-executors"))]
const HEAP_BYTES: usize = 100 * 1024;
const HIL_LOW_MODE_BUILD: bool = option_env!("OWB_HIL_LOW_MODE").is_some();
const OWB_LOG_LEVEL: &str = match option_env!("OWB_LOG_LEVEL") {
    Some(level) => level,
    None => "info",
};

fn configured_log_level() -> LevelFilter {
    if OWB_LOG_LEVEL.eq_ignore_ascii_case("off") {
        LevelFilter::Off
    }
    else if OWB_LOG_LEVEL.eq_ignore_ascii_case("error") {
        LevelFilter::Error
    }
    else if OWB_LOG_LEVEL.eq_ignore_ascii_case("warn") {
        LevelFilter::Warn
    }
    else if OWB_LOG_LEVEL.eq_ignore_ascii_case("debug") {
        LevelFilter::Debug
    }
    else if OWB_LOG_LEVEL.eq_ignore_ascii_case("trace") {
        LevelFilter::Trace
    }
    else {
        LevelFilter::Info
    }
}

fn init_profile_i2c<'d>(
    i2c0: esp_hal::peripherals::I2C0<'d>,
    profile: &profile::BoardProfile,
) -> Bus<'d> {
    // The selected board profile owns the primary I2C bus wiring, so new
    // boards only need profile updates instead of boot-path edits.
    I2c::new(i2c0, I2cConfig::default())
        .expect("I2C init")
        .with_sda(unsafe { AnyPin::steal(profile.i2c_bus.sda) })
        .with_scl(unsafe { AnyPin::steal(profile.i2c_bus.scl) })
}

fn spawn_runtime(
    spawner: embassy_executor::Spawner,
    stack: Stack<'static>,
    ap_ip: Option<Ipv4Address>,
    has_camera: bool,
) {
    // Gate with #[cfg] (not cfg!()) so the linker can gc ws_task_minimal_0's
    // TaskPool static in non-real-executors builds.
    #[cfg(feature = "real-executors")]
    if HIL_LOW_MODE_BUILD {
        owb_core::runtime::spawn_common_minimal_single(spawner, stack, ap_ip);
        return;
    }

    // Gate the camera branch at compile time so ws_task_0/ws_task_1 from
    // owb-core runtime are eliminated by --gc-sections in no-camera builds.
    // Without this gate the linker keeps those TaskPool statics in DRAM even
    // when has_camera is always false, exhausting the DRAM budget.
    #[cfg(not(feature = "no-camera"))]
    if has_camera {
        owb_core::runtime::spawn_camera_web_app(spawner, stack, ap_ip);
        camera_stream_task(stack)
            .map(|t| spawner.spawn(t))
            .unwrap_or_else(|_| esp_println::println!("[ERR] camera_stream_task spawn failed"));
        camera_snap_task()
            .map(|t| spawner.spawn(t))
            .unwrap_or_else(|_| esp_println::println!("[ERR] camera_snap_task spawn failed"));
        return;
    }

    // Normal / no-camera path: one PWA HTTP/WS worker.
    ui_assets::spawn_pwa_runtime(spawner, stack, ap_ip);
}

fn attach_board_runtime(
    spawner: embassy_executor::Spawner,
    profile: &'static profile::BoardProfile,
    _capabilities: board::BoardCapabilities,
) {
    // Spawn failures are surfaced as panics (I-05): a missing board runtime
    // means the command bus executor is absent and motion commands are lost.
    #[cfg(feature = "real-executors")]
    {
        real_runtime_task(profile, _capabilities)
            .map(|t| spawner.spawn(t))
            .expect("critical: real_runtime_task spawn failed");
        return;
    }

    #[cfg(not(feature = "real-executors"))]
    match profile.runtime {
        BoardRuntimeKind::CameraOnly | BoardRuntimeKind::DevKitReference => {
            shadow_runtime_task(profile.runtime)
                .map(|t| spawner.spawn(t))
                .expect("critical: shadow_runtime_task spawn failed");
        }
        BoardRuntimeKind::OwbS3V1 | BoardRuntimeKind::OwbS3V2 => {
            shadow_runtime_task(profile.runtime)
                .map(|t| spawner.spawn(t))
                .expect("critical: shadow_runtime_task spawn failed");
        }
        BoardRuntimeKind::HostSimulation => unreachable!(),
    }
}

#[cfg(target_arch = "xtensa")]
fn maybe_start_dual_core_board_runtime(
    cpu_ctrl: esp_hal::peripherals::CPU_CTRL,
    profile: &'static profile::BoardProfile,
    capabilities: board::BoardCapabilities,
) -> bool {
    if profile.execution_plan.topology != CpuTopology::DualCore {
        return false;
    }

    if profile.execution_plan.control_cpu != 1 || profile.execution_plan.network_cpu != 0 {
        esp_println::println!(
            "[SYS] dual-core policy unsupported ctrl={} net={}, fallback single-core",
            profile.execution_plan.control_cpu,
            profile.execution_plan.network_cpu
        );
        return false;
    }

    static CONTROL_CORE_STACK: StaticCell<CpuStack<CONTROL_CORE_STACK_BYTES>> = StaticCell::new();
    static CONTROL_CORE_EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();

    let control_stack = CONTROL_CORE_STACK
        .uninit()
        .write(CpuStack::<CONTROL_CORE_STACK_BYTES>::new());

    esp_println::println!("[SYS] dual-core split active: control runtime pinned to APP core");

    esp_rtos::start_second_core(
        cpu_ctrl,
        unsafe { SoftwareInterrupt::<1>::steal() },
        control_stack,
        move || {
            let executor = CONTROL_CORE_EXECUTOR
                .uninit()
                .write(esp_rtos::embassy::Executor::new());
            executor.run(move |spawner| {
                attach_board_runtime(spawner, profile, capabilities);
            });
        },
    );

    true
}

// Wi‑Fi credentials and AP fallback
const SSID: &str = match option_env!("SSID") {
    Some(v) => v,
    None => "",
};
const PASSWORD: &str = match option_env!("PASSWORD") {
    Some(v) => v,
    None => "",
};
const AP_SSID: &str = match option_env!("AP_SSID") {
    Some(v) => v,
    None => "OWB-BOT",
};
const AP_PASSWORD: &str = match option_env!("AP_PASSWORD") {
    Some(v) => v,
    None => "",
};
// Pre-shared control secret (C1). When set at build time, every control
// transport (WebSocket today, BLE next) requires it before accepting commands.
// Empty = unauthenticated control surface (dev/bench only).
const CONTROL_KEY: &str = match option_env!("CONTROL_KEY") {
    Some(v) => v,
    None => "",
};
// Safety override (C2): an open (no-password) SoftAP is refused by default
// because AP membership is not authentication on a motor-driving robot. Set
// OWB_ALLOW_OPEN_AP=1 only for bench bring-up.
const ALLOW_OPEN_AP: bool = option_env!("OWB_ALLOW_OPEN_AP").is_some();
// Safety override (C3): when CONTROL_KEY is unset the control surface is
// closed by default (no WS commands accepted). Set OWB_ALLOW_NO_KEY=1 only
// for bench/dev sessions where you accept unauthenticated motor commands.
const ALLOW_NO_KEY: bool = option_env!("OWB_ALLOW_NO_KEY").is_some();

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger(configured_log_level());
    let esp_config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(esp_config);
    #[cfg(target_arch = "xtensa")]
    let dual_core_cpu_ctrl = peripherals.CPU_CTRL;
    esp_alloc::heap_allocator!(size: HEAP_BYTES);
    // Add DRAM2 as a second heap region.  DRAM2 (0x3FCDB700, 72 KB) is reserved
    // by memory.x specifically for application heap; adding it here keeps DRAM1
    // heap small enough that the CPU0 main-task stack has room for the 14 KB
    // Box::pin(run_with_router) temporary on first poll.
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            core::ptr::addr_of_mut!(DRAM2_HEAP_MEM) as *mut u8,
            DRAM2_HEAP_BYTES,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
    let profile = profile::select_profile();

    {
        #[cfg(target_arch = "xtensa")]
        {
            use esp_hal::{
                interrupt::software::SoftwareInterruptControl,
                timer::systimer::SystemTimer,
            };
            let systimer = SystemTimer::new(peripherals.SYSTIMER);
            // esp-rtos 0.3 requires a SoftwareInterrupt<0> for context switches on
            // every arch (previously Xtensa took only the timer).
            let software = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
            esp_rtos::start(systimer.alarm0, software.software_interrupt0);
        }
        #[cfg(target_arch = "riscv32")]
        {
            use esp_hal::{
                interrupt::software::SoftwareInterruptControl,
                timer::systimer::SystemTimer,
            };
            let systimer = SystemTimer::new(peripherals.SYSTIMER);
            let software = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
            esp_rtos::start(systimer.alarm0, software.software_interrupt0);
        }
    }

    // ── Hardware watchdog (RWDT) ────────────────────────────────────────────
    // Resets the entire system if the feed task fails to run within 5 s.
    // This catches executor hangs that the software deadman watchdog cannot —
    // e.g. a blocking I2C read holding the bus, or a WiFi driver stall.
    // Feed interval is 2 s (half the timeout) for a comfortable margin.
    // Brownout detection is configured by the bootloader (esp-bootloader-esp-idf);
    // esp-hal 1.1 does not expose a runtime brownout API for ESP32-S3.
    {
        static RWDT_CELL: StaticCell<Rwdt> = StaticCell::new();
        let mut rtc = Rtc::new(peripherals.LPWR);
        rtc.rwdt.set_timeout(
            RwdtStage::Stage0,
            esp_hal::time::Duration::from_millis(5_000),
        );
        rtc.rwdt.enable();
        let rwdt_ref = RWDT_CELL.init(rtc.rwdt);
        // I-05: surface spawn failure — without this task the RWDT fires in 5 s.
        watchdog_feed_task(rwdt_ref)
            .map(|t| spawner.spawn(t))
            .expect("critical: watchdog_feed_task spawn failed");
        esp_println::println!("[WDT] RWDT armed: 5 s system reset if feed stalls");
    }

    esp_println::println!(
        "[SYS] profile: {} ({:?}, {:?}) exec={:?} ctrl={} net={} media={}",
        profile.name,
        profile.family,
        profile.runtime,
        profile.execution_plan.topology,
        profile.execution_plan.control_cpu,
        profile.execution_plan.network_cpu,
        profile.execution_plan.media_cpu
    );

    let mut app_has_camera = false;

    {
        let mut i2c = init_profile_i2c(peripherals.I2C0, profile);
        if profile.has_camera && !cfg!(feature = "no-camera") {
            match camera::init(&mut i2c, peripherals.LCD_CAM, peripherals.DMA_CH0, profile).await {
                Ok(()) => {
                    app_has_camera = true;
                    esp_println::println!("[CAM] init ok");
                    // Spawn snap task here so it runs regardless of WiFi availability.
                    // In spawn_runtime it is also spawned (WiFi path) — the second
                    // spawn attempt is silently dropped because the TaskPool is full.
                    #[cfg(not(feature = "no-camera"))]
                    camera_snap_task()
                        .map(|t| spawner.spawn(t))
                        .unwrap_or_else(|_| {
                            esp_println::println!("[ERR] camera_snap_task spawn failed")
                        });
                }
                Err(error) => {
                    esp_println::println!(
                        "[CAM] init failed on profile {}: {:?}; continuing without camera",
                        profile.name,
                        error
                    );
                }
            }
        }
        else {
            esp_println::println!(
                "[CAM] disabled for profile {} (profile_has_camera={} no_camera_feature={})",
                profile.name,
                profile.has_camera,
                cfg!(feature = "no-camera")
            );
        }
        let mut capabilities = board::boot_probe(&mut i2c, profile);
        capabilities.has_camera = app_has_camera;
        #[cfg(target_arch = "xtensa")]
        let board_runtime_attached =
            maybe_start_dual_core_board_runtime(dual_core_cpu_ctrl, profile, capabilities);
        #[cfg(not(target_arch = "xtensa"))]
        let board_runtime_attached = false;

        if !board_runtime_attached {
            attach_board_runtime(spawner, profile, capabilities);
        }
    }

    // ---- Control-channel auth (C1) ----
    // Configure the shared control secret before any transport can accept a
    // connection. Done here so WebSocket and the forthcoming BLE GATT service
    // both gate on the same check.
    owb_core::net::auth::set_control_secret(CONTROL_KEY).await;
    if CONTROL_KEY.is_empty() {
        if ALLOW_NO_KEY {
            owb_core::net::auth::set_open_surface();
            esp_println::println!(
                "[SEC] WARNING: open control surface (OWB_ALLOW_NO_KEY set; bench/dev only)"
            );
        }
        else {
            esp_println::println!(
                "[SEC] CONTROL_KEY unset; WebSocket commands DISABLED. \
                 Set CONTROL_KEY=<secret> to enable, or OWB_ALLOW_NO_KEY=1 for bench use."
            );
        }
    }
    else {
        esp_println::println!("[SEC] control secret configured; clients must present ?key=…");
    }

    // ---- Offline control over USB-Serial-JTAG ----
    // Lets the bot be driven over the USB cable with no network at all: send
    // newline-delimited JSON command frames to the serial port. Shares the
    // peripheral with esp-println's log output (independent rx/tx FIFOs).
    let usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
    // I-05: log spawn failure rather than silently dropping it.
    usb_command_task(usb_serial)
        .map(|t| spawner.spawn(t))
        .unwrap_or_else(|_| esp_println::println!("[ERR] usb_command_task spawn failed"));

    // ---- Wi‑Fi + Network bring-up (STA, SoftAP fallback) ----
    // esp-radio 1.0: the controller is created standalone and the network
    // interfaces are independent singletons (`Interface::station` /
    // `Interface::access_point`) that implement the embassy-net `Driver`.
    // `WifiController::new` performs radio init internally — the esp-rtos
    // scheduler started above provides the preempt driver it requires — so there
    // is no separate `esp_radio::init()` / `Controller` handle anymore.
    let mut wifi_ctrl = WifiController::new(peripherals.WIFI, ControllerConfig::default())
        .expect("wifi controller");

    static NET_RESOURCES: StaticCell<StackResources<S3_NET_SOCKETS>> = StaticCell::new();
    static NET_STACK: StaticCell<Stack<'static>> = StaticCell::new();
    let rng = esp_hal::rng::Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let mut started_ws = false;
    if !SSID.is_empty() {
        let sta_cfg = StationConfig::default()
            .with_ssid(SSID)
            .with_password(PASSWORD.into());
        // `set_config` applies the config and starts the interface (esp_wifi_start
        // runs inside it), so there is no separate start step.
        if wifi_ctrl
            .set_config(&WifiModeConfig::Station(sta_cfg))
            .is_ok()
            && wifi_ctrl.connect_async().await.is_ok()
        {
            let sta = Interface::station();
            let net_cfg = NetConfig::dhcpv4(Default::default());
            let (stack, runner) =
                owb_core::net::net::build_stack(sta, net_cfg, &NET_RESOURCES, seed);
            let stack = NET_STACK.init(stack);
            net_task(runner)
                .map(|t| spawner.spawn(t))
                .expect("critical: net_task spawn failed (STA)");
            stack.wait_config_up().await;
            esp_println::println!("Wi‑Fi STA up, IP assigned");
            esp_println::println!("[HEAP] before spawn_runtime:\n{}", esp_alloc::HEAP.stats());
            spawn_runtime(spawner, *stack, None, app_has_camera);
            started_ws = true;
        }
        else {
            esp_println::println!("STA connect failed; falling back to SoftAP");
        }
    }

    if !started_ws {
        // C2: refuse to bring up an open AP unless explicitly allowed. On an
        // actuated robot, an open AP + the (optional) control auth means anyone
        // in radio range could drive it, so WPA2 is the default and the open
        // path is a logged, opt-in bench mode.
        let ap_cfg = if !AP_PASSWORD.is_empty() {
            Some(
                AccessPointConfig::default()
                    .with_ssid(AP_SSID)
                    .with_auth_method(AuthenticationMethod::Wpa2Personal)
                    .with_password(AP_PASSWORD.into()),
            )
        }
        else if ALLOW_OPEN_AP {
            esp_println::println!(
                "[SEC] WARNING: starting OPEN SoftAP (OWB_ALLOW_OPEN_AP set; bench use only)"
            );
            Some(
                AccessPointConfig::default()
                    .with_ssid(AP_SSID)
                    .with_auth_method(AuthenticationMethod::None),
            )
        }
        else {
            None
        };

        match ap_cfg {
            Some(cfg) => {
                let ap = Interface::access_point();
                let (stack_obj, runner, ap_ip) =
                    owb_core::runtime::bringup_ap_default(ap, &NET_RESOURCES, seed);
                let stack = NET_STACK.init(stack_obj);
                net_task(runner)
                    .map(|t| spawner.spawn(t))
                    .expect("critical: net_task spawn failed (SoftAP)");
                // `set_config(AccessPoint)` internally stops STA mode (if a failed
                // STA attempt left it running) and starts the SoftAP, so the
                // STA -> AP transition needs no explicit stop in esp-radio 1.0.
                if wifi_ctrl
                    .set_config(&WifiModeConfig::AccessPoint(cfg))
                    .is_ok()
                {
                    esp_println::println!("SoftAP started at 192.168.4.1");
                }
                else {
                    esp_println::println!("SoftAP start failed");
                }
                spawn_runtime(spawner, *stack, Some(ap_ip), app_has_camera);
            }
            None => {
                esp_println::println!(
                    "[SEC] refusing to start open SoftAP: set AP_PASSWORD (WPA2) or \
                     OWB_ALLOW_OPEN_AP=1 for bench use. No control surface started."
                );
            }
        }
    }

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

#[cfg(not(feature = "no-camera"))]
#[embassy_executor::task]
async fn camera_stream_task(stack: Stack<'static>) -> ! {
    owb_core::net::server::run_camera_stream(
        3,
        owb_core::net::server::DEFAULT_MEDIA_PORT,
        stack,
        None,
        camera::stream,
    )
    .await
}

#[cfg(not(feature = "no-camera"))]
#[embassy_executor::task]
async fn camera_snap_task() -> ! { camera::snap_task().await }

/// Feeds the RWDT every 2 s (half the 5 s timeout).
///
/// Runs as long as the Embassy executor is alive. If this task stops being
/// scheduled — executor hang, priority inversion, or catastrophic task failure —
/// the RWDT fires and resets the system, preventing a stuck PWM output.
#[embassy_executor::task]
async fn watchdog_feed_task(rwdt: &'static mut Rwdt) -> ! {
    loop {
        Timer::after(Duration::from_millis(2_000)).await;
        rwdt.feed();
    }
}

#[embassy_executor::task]
async fn shadow_runtime_task(runtime: BoardRuntimeKind) -> ! {
    let mut executor = board::ShadowBoardRuntime::new(runtime);
    owb_core::node::command_bus::run_command_bus(&mut executor).await
}

#[cfg(feature = "real-executors")]
#[embassy_executor::task]
async fn real_runtime_task(
    profile: &'static profile::BoardProfile,
    capabilities: board::BoardCapabilities,
) -> ! {
    let mut executor = board::RealBoardRuntime::from_profile(profile, capabilities);
    owb_core::node::command_bus::run_command_bus(&mut executor).await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Interface>) { runner.run().await; }

/// Offline control over the USB-Serial-JTAG link: read newline-delimited JSON
/// command frames (the same envelope the WebSocket accepts, e.g.
/// `{"ct":"l","lc":"breath","r":0,"g":0,"b":255}` or
/// `{"ct":"i","ic":"probe_speed","wheel":0,"speed_norm":0.2}`) and push them to
/// the shared command bus. Polls `drain_rx_fifo` (never blocks, no interrupt
/// wiring) and yields between reads, so it coexists with esp-println's TX
/// logging on the same peripheral. No network required.
///
/// ## Auth model (C1): USB is implicitly trusted
///
/// Unlike the WebSocket transport, this task does NOT call
/// [`owb_core::net::auth::verify_control_secret`] before queuing
/// commands. That is a deliberate policy decision, not an oversight: USB-Serial-
/// JTAG is a physically-local, wired transport. Anyone able to send bytes on it
/// already has hands-on physical access to the robot (they are holding the
/// cable), at which point the network-facing `CONTROL_KEY` adds no security and
/// would only break headless bench/offline bring-up. See
/// `owb_core::net::auth::USB_IS_TRUSTED` for the canonical statement of
/// this policy. The remote transports (WebSocket today, BLE next) remain gated
/// on the shared control secret.
#[embassy_executor::task]
async fn usb_command_task(usb: UsbSerialJtag<'static, Blocking>) -> ! {
    use owb_core::{
        ctrl::{sysctrl::I2CCommand, SystemCommand},
        node::command_bus::{CommandEnvelope, COMMAND_BUS},
    };

    // `drain_rx_fifo` lives on the Rx half; the Tx half stays with esp-println.
    let (mut rx, _tx) = usb.split();
    let mut chunk = [0u8; 64];
    let mut line = [0u8; 256];
    let mut len = 0usize;

    esp_println::println!("[USB] serial command input ready (newline-delimited JSON)");
    loop {
        let n = rx.drain_rx_fifo(&mut chunk);
        if n == 0 {
            Timer::after(Duration::from_millis(15)).await;
            continue;
        }
        for &byte in &chunk[..n] {
            match byte {
                b'\n' | b'\r' => {
                    if len > 0 {
                        match core::str::from_utf8(&line[..len]) {
                            Ok(text) => match owb_core::net::protocol::parse_command_from_str(text)
                            {
                                Ok(cmd) => {
                                    let envelope = CommandEnvelope {
                                        cmd,
                                        expects_reply: false,
                                        received_ms: embassy_time::Instant::now().as_millis(),
                                    };
                                    // High-rate motion-stream frames (T/Y/O) are
                                    // intentionally droppable: the bus coalesces
                                    // them and the deadman watchdog covers a stall,
                                    // so a full queue is fine. Every other command
                                    // (enable/disable/estop, LED, power) is a
                                    // discrete control action that must never be
                                    // silently lost, so it uses the blocking send.
                                    let is_motion_stream = matches!(
                                        envelope.cmd,
                                        SystemCommand::I(
                                            I2CCommand::T { .. }
                                                | I2CCommand::Y { .. }
                                                | I2CCommand::O { .. }
                                        )
                                    );
                                    if is_motion_stream {
                                        if COMMAND_BUS.try_send(envelope).is_err() {
                                            esp_println::println!(
                                                "[USB] motion bus full; dropped (coalesced)"
                                            );
                                        }
                                        else {
                                            esp_println::println!("[USB] command accepted");
                                        }
                                    }
                                    else {
                                        COMMAND_BUS.send(envelope).await;
                                        esp_println::println!("[USB] command accepted");
                                    }
                                }
                                Err(_) => esp_println::println!("[USB] unrecognized command"),
                            },
                            Err(_) => esp_println::println!("[USB] non-utf8 input ignored"),
                        }
                        len = 0;
                    }
                }
                _ => {
                    if len < line.len() {
                        line[len] = byte;
                        len += 1;
                    }
                    else {
                        // Overrun: drop the partial line and resync on next newline.
                        len = 0;
                    }
                }
            }
        }
    }
}
