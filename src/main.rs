#![no_std]
#![no_main]

// Module imports
mod utils;
// Standard imports
use core::cell::RefCell;
// Embassy framework imports
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
// ESP-specific imports
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    i2c::master::{Config, I2c},
    rmt::{Channel, Rmt},
    rng::Rng,
    timer::timg::TimerGroup,
    Blocking,
};
use esp_hal::clock::CpuClock;
use esp_hal::time::RateExtU32;
use esp_wifi::{
    init,
    wifi::{
        ClientConfiguration,
        Configuration,
        WifiController,
        WifiDevice,
        WifiEvent,
        WifiStaDevice,
        WifiState,
    },
    EspWifiController,
};
use log::LevelFilter;
//Internal Modules
use utils::{
    connection::websocket_server,
    controllers::{I2CDevices, I2C_CHANNEL, LedModule, LED_CHANNEL},
    packages::{SmartLedsAdapter}
};

// Static memory allocation macro
macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

// Constants for Wi-Fi credentials
const SSID: &str = "Unavailable";
const PASSWORD: &str = "home4226101";
const BUF_SIZE: usize = 2 * 24 + 1;

// Static memory for I2C devices
static I2C_REF: static_cell::StaticCell<RefCell<I2c<'static, Blocking>>> =
    static_cell::StaticCell::new();
static DEVICES: static_cell::StaticCell<Option<I2CDevices<'static, I2c<'static, Blocking>>>> =
    static_cell::StaticCell::new();


#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> !
{
    esp_println::logger::init_logger(LevelFilter::Trace);
    tracing::info!("Logger initialized");

    // Initialize peripherals
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let mut rng = Rng::new(peripherals.RNG);

    // Initialize I2C
    let i2c = I2c::new(peripherals.I2C0, Config::default()).unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);
    let i2c_ref = I2C_REF.init(RefCell::new(i2c));

    let devices_init = I2CDevices::new(i2c_ref, 0.148, 0.195);
    if let Err(e) = &devices_init {
        tracing::warn!("Skipping I2C/IMU usage: {:?}", e);
    }
    let devices = DEVICES.init(devices_init.ok());

    // IInitialize LEDs
    let rmt = Rmt::new(peripherals.RMT, 80.MHz()).unwrap();
    let rmt_buffer = smartLedBuffer!(2);
    let adapter = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO12, rmt_buffer);
    let leds = LedModule::new(adapter);

    // Initialize heap allocator
    esp_alloc::heap_allocator!(72 * 1024);

    // Set up TimerGroup for Wi-Fi initialization
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

    // Initialize Wi-Fi
    let init = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();

    // Initialize network stack
    let config = embassy_net::Config::dhcpv4(Default::default());

    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    // Spawn tasks for network and WebSocket handling
    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(runner)).ok();

    // Wait for network connection
    wait_for_network(&stack).await;

    tracing::info!("Spawning controllers");
    // Spawn a task for handling I2C messages
    spawner.spawn(handle_i2c_message(devices)).unwrap();

    spawner.spawn(handle_leds_message(leds)).unwrap();

    // Run the WebSocket server
    websocket_server(0, 82, stack, None).await;
}

/// Task to manage Wi-Fi connections
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>)
{
    tracing::info!("Start connection task");
    tracing::info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await;
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
            tracing::info!("Wi-Fi started!");
        }

        match controller.connect_async().await {
            Ok(_) => tracing::info!("Wi-Fi connected!"),
            Err(e) => {
                tracing::info!("Failed to connect: {e:?}");
                Timer::after(Duration::from_millis(5000)).await;
            }
        }
    }
}

/// Task to manage the network stack
#[embassy_executor::task]
async fn net_task(mut runner:Runner<'static, WifiDevice<'static, WifiStaDevice>>) { runner.run().await; }

/// Task to handle I2C messages
#[embassy_executor::task]
pub async fn handle_i2c_message(
    devices: &'static mut Option <I2CDevices<'static, I2c<'static, Blocking>>>
) -> !
{
    loop {
        let cmd = I2C_CHANNEL.receiver().receive().await;
        tracing::info!("Received I2CCommand: {:?}", cmd);

        if let Some(devices) = devices.as_mut() {
            // If we have a valid I2CDevices, execute the command
            match devices.execute_command(cmd) {
                Ok(Some((accel, gyro, temp))) => {
                    tracing::info!(?accel, ?gyro, ?temp, "IMU Data Read");
                }
                Ok(None) => {
                    tracing::info!("I2C command executed successfully");
                }
                Err(err) => {
                    tracing::error!(?err, "Failed to execute I2C command");
                }
            }
        } else {
            // If I2CDevices is None, just warn or ignore
            tracing::warn!("I2C command received but devices not initialized: {cmd:?}");
        }
    }
}

/// Task to handle LEDs messages
#[embassy_executor::task]
pub async fn handle_leds_message(
    mut leds: LedModule<SmartLedsAdapter<Channel<Blocking, 0>, { BUF_SIZE }>>
) -> !
{
    loop {
        let cmd = LED_CHANNEL.receiver().receive().await;
        tracing::info!("Received LEDCommand: {:?}", cmd);

        match leds.ex_command(cmd) {
            Ok(()) => tracing::info!("Command executed successfully"),
            Err(err) => tracing::error!(?err, "Failed to execute LEDs command"),
        }
    }
}

/// Helper function to wait for network connection
async fn wait_for_network(stack: &Stack<'static>)
{
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    tracing::info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            tracing::info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}
