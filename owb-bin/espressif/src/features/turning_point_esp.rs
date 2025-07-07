#![no_std]
#![no_main]
extern crate alloc;

// Standard imports
use core::cell::RefCell;

// ESP-specific imports
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    i2c::master::{Config, I2c},
    rng::Rng,
    timer::timg::TimerGroup,
    Blocking,
};
use esp_wifi::{
    init,
    wifi::{WifiController, WifiDevice},
    EspWifiController,
};
use log::LevelFilter;


// Internal Modules
use omni_wheel::{mk_static, utils};

// Constants for Wi-Fi credentials
const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

type Bus = I2c<'static, Blocking>;

// Static memory for I2C devices
static I2C_BUS: static_cell::StaticCell<RefCell<I2c<'static, Blocking>>> =
    static_cell::StaticCell::new();

#[esp_hal_embassy::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger(LevelFilter::Trace);
    tracing::info!("Logger initialized");

    let esp_config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(esp_config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut rng = Rng::new(peripherals.RNG);
    let sda_per = peripherals.GPIO21;
    let scl_per = peripherals.GPIO2;

    let i2c_bus = I2C_BUS.init(RefCell::new(
        I2c::new(peripherals.I2C0, Config::default())
            .unwrap()
            .with_sda(sda_per)
            .with_scl(scl_per),
    ));

    let sys_ctrl = utils::SystemController::new(i2c_bus, None, None);

    // Wi-Fi Configuration Block
    // *******************************************************************
    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );

    let (controller, interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

    let wifi_sta_device = interfaces.sta;
    let dhcpv_config = {
        let mut dhcpv = embassy_net::DhcpConfig::default();
        dhcpv.server_port = 67;
        dhcpv.client_port = 68;
        dhcpv
    };

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let timg1 = TimerGroup::new(peripherals.TIMG1);
            esp_hal_embassy::init(timg1.timer0);
        } else {
            use esp_hal::timer::systimer::SystemTimer;
            let systimer = SystemTimer::new(peripherals.SYSTIMER);
            esp_hal_embassy::init(systimer.alarm0);
        }
    }

    let sta_config = embassy_net::Config::dhcpv4(dhcpv_config);
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stack
    let (sta_stack, sta_runner) = embassy_net::new(
        wifi_sta_device,
        sta_config,
        mk_static!(embassy_net::StackResources<3>, embassy_net::StackResources::<3>::new()),
        seed,
    );
    // End of Wi-Fi Configuration Block
    // ************************************************************

    spawner.spawn(i2c_task(sys_ctrl)).ok();
    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(sta_runner)).ok();

    wait_for_network(&sta_stack).await;

    utils::wss(0, 80, sta_stack, None).await;
}

#[embassy_executor::task]
async fn i2c_task(mut ctrl: utils::SystemController<Bus>) -> ! {
    utils::SystemController::i2c_ch(&mut ctrl).await
}
/// Task to manage Wi-Fi connections
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    use esp_wifi::wifi::{ClientConfiguration, Configuration, WifiEvent, WifiState};
    tracing::info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                utils::Timer::after(utils::Duration::from_millis(5000)).await;
            }
            _ => {}
        }

        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            controller.start_async().await.unwrap();
            tracing::info!("Connecting to SSID: {}", SSID);
        }

        match controller.connect_async().await {
            Ok(_) => tracing::info!("Wi-Fi connected!"),
            Err(e) => {
                tracing::info!("Failed to connect: {e:?}");
                utils::Timer::after(utils::Duration::from_millis(5000)).await;
            }
        }
    }
}

/// Task to manage the network stack
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

/// Helper function to wait for network connection
async fn wait_for_network(stack: &embassy_net::Stack<'static>) {
    loop {
        if stack.is_link_up() {
            break;
        }
        utils::Timer::after(utils::Duration::from_millis(500)).await;
    }

    tracing::info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            tracing::info!("Got IP: {}", config.address);
            break;
        }
        utils::Timer::after(utils::Duration::from_millis(500)).await;
    }
}
