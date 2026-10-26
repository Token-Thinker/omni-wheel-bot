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
#[cfg(target_arch = "xtensa")]
use esp_hal::system::Stack as CpuStack;
use esp_hal::{
    clock::CpuClock,
    gpio::AnyPin,
    i2c::master::{Config as I2cConfig, I2c},
    rtc_cntl::{Rtc, Rwdt, RwdtStage},
    usb::usb_serial_jtag::UsbSerialJtag,
    Blocking,
};
use esp_radio::{
    self,
    wifi::{
        ap::AccessPointConfig,
        sta::StationConfig,
        AuthenticationMethodConfig,
        Config as WifiModeConfig,
        ControllerConfig,
        Interface,
        Password,
        Ssid,
        WifiController,
    },
};
use log::LevelFilter;
// `BoardRuntimeKind` is only referenced by the shadow runtime, which is
// compiled out when the real GPIO executors are enabled.
#[cfg(not(feature = "real-executors"))]
use owb_core::platform::BoardRuntimeKind;
use owb_core::{ext::prelude::*, platform::CpuTopology};

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
//    The CPU0 main-task stack is whatever DRAM1 is left after statics, and
//    `heap_allocator!` puts the DRAM1 heap in .bss, so HEAP_BYTES comes straight
//    out of the stack. Read the real figure from the ELF, not from arithmetic:
//    stack = `_stack_start_cpu0` - `_stack_end_cpu0`.
//
//    Measured 2026-10-24: with HEAP_BYTES = 100 KB the stack was only ~8-9 KB
//    (the old "56 KB" estimate predates the 40 KB camera DMA buffer, the
//    camera task pools and the core-1 stack). That was a latent overflow:
//    the stack grows down into the top of .bss, which holds the Wi-Fi blob's
//    supplicant/hostapd state (`gWpaSm`, `global_hapd`, `eloop`, ...). esp-hal
//    1.2's stack guard turned it into a boot panic while formatting a float.
//    72 KB gives the stack ~36 KB -- room for the 14 KB
//    Box::pin(run_with_router) temporary on first poll plus deep `fmt` calls.
//
//  • DRAM2 (dram2_seg) 0x3FCDB700 – 0x3FCED710  72 KB
//    Reserved by memory.x for application heap use.  We add 70 KB here,
//    leaving ~2 KB as a safety margin below the 72 KB segment limit.
//
//  Internal heap = 72 KB + 70 KB = 142 KB, registered before PSRAM, so the
//  allocator fills internal RAM first and only spills general allocations
//  into the 16 MB PSRAM region once it is full. Wi-Fi DMA buffers request
//  internal memory explicitly and never spill.

const DRAM2_HEAP_BYTES: usize = 70 * 1024;

#[link_section = ".dram2_uninit"]
static mut DRAM2_HEAP_MEM: core::mem::MaybeUninit<[u8; DRAM2_HEAP_BYTES]> =
    core::mem::MaybeUninit::uninit();

/// DRAM1 heap. Both `real-executors` and the stub build used to declare this
/// separately under opposing `#[cfg]`s with the identical value, which read as
/// if the two builds were deliberately tuned apart. They are not -- the DRAM1
/// budget above is a property of the linker layout, not of the executor choice.
const HEAP_BYTES: usize = 72 * 1024;
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

    // Every board serves the Leptos operator UI on port 80. Camera boards used
    // to get owb-core's camera app shell there instead, whose `/` is the
    // compact HIL test page -- so the phone UI was never served on v1-alpha,
    // and that page also never sent the control key (every WebSocket was
    // rejected once CONTROL_KEY was set). The MJPEG stream keeps its own
    // port (81).
    ui_assets::spawn_pwa_runtime(spawner, stack, ap_ip);

    // `#[cfg]`, not `cfg!()`, so no-camera builds drop the stream task's pool.
    #[cfg(not(feature = "no-camera"))]
    if has_camera {
        camera_stream_task(stack)
            .map(|t| spawner.spawn(t))
            .unwrap_or_else(|_| esp_println::println!("[ERR] camera_stream_task spawn failed"));
        // camera_snap_task is already running: it is spawned at camera init so
        // `snap` works without Wi-Fi.
    }
    #[cfg(feature = "no-camera")]
    let _ = has_camera;
}

fn attach_board_runtime(
    spawner: embassy_executor::Spawner,
    profile: &'static profile::BoardProfile,
    _capabilities: board::BoardCapabilities,
) {
    // Spawn failures are surfaced as panics (I-05): a missing board runtime
    // means the command bus executor is absent and motion commands are lost.
    //
    // No `return` after the real-executors arm: the shadow `match` below is
    // `#[cfg]`-ed out in that configuration, so there is nothing to fall into.
    #[cfg(feature = "real-executors")]
    real_runtime_task(profile, _capabilities)
        .map(|t| spawner.spawn(t))
        .expect("critical: real_runtime_task spawn failed");

    // All four board runtimes take the identical shadow path — they were
    // previously two arms with byte-identical bodies. Kept enumerated rather
    // than collapsed to `_` so that adding a `BoardRuntimeKind` is a compile
    // error here instead of silently landing in the `unreachable!` arm.
    #[cfg(not(feature = "real-executors"))]
    match profile.runtime {
        BoardRuntimeKind::CameraOnly
        | BoardRuntimeKind::DevKitReference
        | BoardRuntimeKind::OwbS3V1
        | BoardRuntimeKind::OwbS3V2 => {
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
        // esp-rtos 0.4 takes the FROM_CPU_INTR1 peripheral directly. `main`
        // has already moved `peripherals`, so steal the singleton; nothing
        // else uses this software interrupt.
        unsafe { esp_hal::peripherals::FROM_CPU_INTR1::steal() },
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

    // External PSRAM (ESP32-S3-WROOM-2 octal SPI) as an additional heap region.
    // Unlocks large allocations that don't fit in internal SRAM — e.g. a
    // higher-resolution camera frame buffer (the internal DMA capture buffer is
    // capped at ~40 KB = one 160x120 YUV422 frame). The macro both initializes
    // PSRAM and registers it with esp-alloc as an External-capability region.
    //
    // Must be built in release mode (the workspace esp32s3 alias uses --release).
    // Degrades safely: if no PSRAM chip responds, init returns a 0-length range
    // and the region is simply empty — boot continues on internal RAM.
    //
    // WARNING: atomics are unreliable in S3 PSRAM, so nothing here must place an
    // `Atomic*` in an External allocation (plain byte/frame buffers are fine).
    #[cfg(target_arch = "xtensa")]
    {
        esp_alloc::psram_allocator!(
            peripherals.PSRAM,
            esp_hal::psram,
            esp_hal::psram::PsramConfig {
                mode: esp_hal::psram::PsramMode::OctalSpi,
                ..core::default::Default::default()
            }
        );
        esp_println::println!(
            "[PSRAM] external heap registered (octal SPI); {}",
            esp_alloc::HEAP.stats()
        );
    }

    let profile = profile::select_profile();

    // Xtensa and RISC-V start the scheduler identically: esp-rtos 0.3 requires a
    // SoftwareInterrupt<0> for context switches on *every* arch, where older
    // versions took only the timer on Xtensa. These were two byte-identical
    // `#[cfg]` blocks, which implied a per-arch difference that no longer
    // exists; one block gated on both arches states the actual requirement.
    #[cfg(any(target_arch = "xtensa", target_arch = "riscv32"))]
    {
        use esp_hal::timer::systimer::SystemTimer;
        let systimer = SystemTimer::new(peripherals.SYSTIMER);
        esp_rtos::start(systimer.alarm0, peripherals.FROM_CPU_INTR0);
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
        let mut rtc = Rtc::new(peripherals.RTC_TIMER);
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
                    // Spawn the snap task here so it runs regardless of Wi-Fi
                    // availability; this is its only spawn site.
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

        // Hand I2C0 over explicitly before any board runtime can claim it.
        // The board sensors re-`steal()` I2C0 and reconfigure SDA/SCL on every
        // access; on the dual-core path the control core does that as soon as
        // `start_second_core` returns. Letting this handle live to the end of
        // the block instead would (a) leave both cores able to drive the bus
        // concurrently for the rest of boot, and (b) run *this* handle's `Drop`
        // — tearing the peripheral down — after the control core had already
        // taken it over. Boot probing is finished here, so release it now.
        drop(i2c);

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
        // esp-radio 1.0.0-beta.1 rejects an oversized SSID or password instead
        // of truncating it. Log and fall through to the SoftAP rather than
        // panicking at boot: a robot that reboot-loops is worse than one that
        // comes up on its own AP.
        let sta_cfg = match (Ssid::try_from(SSID), Password::try_from(PASSWORD)) {
            (Ok(ssid), Ok(password)) => Some(
                StationConfig::default()
                    .with_ssid(ssid)
                    .with_authentication(AuthenticationMethodConfig::Wpa2Personal(password)),
            ),
            _ => {
                esp_println::println!(
                    "[WIFI] SSID/PASSWORD rejected (SSID <= 32 bytes, password 8-63 chars)"
                );
                None
            }
        };
        // `set_config` applies the config and starts the interface (esp_wifi_start
        // runs inside it), so there is no separate start step.
        let sta_up = match sta_cfg {
            Some(cfg) => {
                wifi_ctrl.set_config(&WifiModeConfig::Station(cfg)).is_ok()
                    && wifi_ctrl.connect_async().await.is_ok()
            }
            None => false,
        };
        if sta_up {
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
        let auth = if !AP_PASSWORD.is_empty() {
            match Password::try_from(AP_PASSWORD) {
                Ok(password) => Some(AuthenticationMethodConfig::Wpa2Personal(password)),
                Err(_) => {
                    esp_println::println!("[SEC] AP_PASSWORD rejected: WPA2 needs 8-63 chars");
                    None
                }
            }
        }
        else if ALLOW_OPEN_AP {
            esp_println::println!(
                "[SEC] WARNING: starting OPEN SoftAP (OWB_ALLOW_OPEN_AP set; bench use only)"
            );
            Some(AuthenticationMethodConfig::Open)
        }
        else {
            None
        };
        let ap_cfg = match (auth, Ssid::try_from(AP_SSID)) {
            (Some(auth), Ok(ssid)) => Some(
                AccessPointConfig::default()
                    .with_ssid(ssid)
                    .with_authentication(auth),
            ),
            (Some(_), Err(_)) => {
                esp_println::println!("[WIFI] AP_SSID rejected: longer than 32 bytes");
                None
            }
            (None, _) => None,
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
                esp_println::println!(
                    "[HEAP] after AP spawn_runtime:\n{}",
                    esp_alloc::HEAP.stats()
                );
                heap_report_task()
                    .map(|t| spawner.spawn(t))
                    .unwrap_or_else(|_| {
                        esp_println::println!("[ERR] heap_report_task spawn failed")
                    });
                // Log every station join/leave. When a phone "can't load the
                // page", this separates "never associated" from "associated
                // but no TCP" without a sniffer.
                ap_events_task(wifi_ctrl)
                    .map(|t| spawner.spawn(t))
                    .unwrap_or_else(|_| esp_println::println!("[ERR] ap_events_task spawn failed"));
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

#[cfg(not(feature = "real-executors"))]
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

/// One heap snapshot once the web workers have allocated their futures.
#[embassy_executor::task]
async fn heap_report_task() {
    Timer::after(Duration::from_secs(8)).await;
    esp_println::println!("[HEAP] 8 s after AP start:\n{}", esp_alloc::HEAP.stats());
}

#[embassy_executor::task]
async fn ap_events_task(controller: WifiController<'static>) {
    use esp_radio::wifi::ap::EventInfo;
    loop {
        match controller
            .wait_for_access_point_connected_event_async()
            .await
        {
            Ok(EventInfo::Connected(info)) => {
                esp_println::println!("[AP] station joined mac={:02x?} aid={}", info.mac, info.aid)
            }
            Ok(EventInfo::Disconnected(info)) => {
                esp_println::println!("[AP] station left: {:?}", info)
            }
            Err(e) => {
                esp_println::println!("[AP] event wait failed: {:?}", e);
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }
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
        node::command_bus::{CommandEnvelope, COMMAND_BUS, COMMAND_QUERY_LOCK, COMMAND_REPLY_BUS},
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
                                    // Sensor reads produce a CommandReply. Wait for
                                    // it and print it, so the bot is fully usable
                                    // headless over USB with no network — otherwise
                                    // the read executes and the answer is dropped.
                                    // Uses the same COMMAND_QUERY_LOCK the WebSocket
                                    // gateway takes, so the two transports cannot
                                    // both be waiting on the single reply channel.
                                    if owb_core::net::gateway::command_requires_reply(&cmd) {
                                        let _guard = COMMAND_QUERY_LOCK.lock().await;
                                        while COMMAND_REPLY_BUS.try_receive().is_ok() {}
                                        COMMAND_BUS
                                            .send(CommandEnvelope {
                                                cmd,
                                                expects_reply: true,
                                                received_ms: embassy_time::Instant::now()
                                                    .as_millis(),
                                            })
                                            .await;
                                        match embassy_time::with_timeout(
                                            Duration::from_millis(1_000),
                                            COMMAND_REPLY_BUS.receive(),
                                        )
                                        .await
                                        {
                                            Ok(reply) => esp_println::println!(
                                                "[USB_REPLY] {}",
                                                owb_core::net::gateway::command_reply_to_json(
                                                    reply
                                                )
                                            ),
                                            Err(_) => esp_println::println!(
                                                "[USB_REPLY] timeout waiting for reply"
                                            ),
                                        }
                                        len = 0;
                                        continue;
                                    }
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
                                Err(_) => esp_println::println!(
                                    "[USB] unrecognized command ({} bytes): {}",
                                    len,
                                    text
                                ),
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
