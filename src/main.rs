#![no_std]
#![no_main]

// Module imports
mod utils;
use utils::connection::websocket_server;
use utils::controllers::{I2CDevices, CHANNEL};

// Standard imports
use core::cell::RefCell;

// Embassy framework imports
use embassy_executor::Spawner;
use embassy_net::{Stack, StackResources};
use embassy_time::{Duration, Timer};

// ESP-specific imports
use esp_alloc as _;
use esp_hal::{
    prelude::*,
    rng::Rng,
    timer::timg::TimerGroup,
    i2c::master::{Config, I2c},
    Blocking,
};
use esp_wifi::{
    init,
    wifi::{ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState},
    EspWifiController,
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
const SSID: &str = "Test";
const PASSWORD: &str = "Test";

// Static memory for I2C devices
static I2C_REF: static_cell::StaticCell<RefCell<I2c<'static, Blocking>>> = static_cell::StaticCell::new();
static DEVICES: static_cell::StaticCell<I2CDevices<'static, I2c<'static, Blocking>>> = static_cell::StaticCell::new();

// Panic handler
#[panic_handler]
pub fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

/// Main async entry point for the program
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    // Initialize peripherals
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    // Initialize I2C
    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);
    let i2c_ref = I2C_REF.init(RefCell::new(i2c));
    let devices = DEVICES.init(I2CDevices::new(i2c_ref, 0.148, 0.195).expect("Failed to initialize devices"));

    // Initialize heap allocator
    esp_alloc::heap_allocator!(72 * 1024);

    // Set up TimerGroup for Wi-Fi initialization
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    // Initialize Wi-Fi
    let init = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, Rng::new(peripherals.RNG), peripherals.RADIO_CLK).unwrap()
    );
    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) = esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();

    // Initialize network stack
    let stack = &*mk_static!(
        Stack<WifiDevice<'_, WifiStaDevice>>,
        Stack::new(
            wifi_interface,
            embassy_net::Config::dhcpv4(Default::default()),
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            1234 // Random seed
        )
    );

    // Spawn tasks for network and WebSocket handling
    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(&stack)).ok();

    // Wait for network connection
    wait_for_network(&stack).await;

    // Spawn a task for handling I2C messages
    spawner.spawn(handle_message(devices)).unwrap();

    // Run the WebSocket server
    websocket_server(0, 8000, stack, None).await;
}

/// Task to manage Wi-Fi connections
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
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
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await;
}

/// Task to handle I2C messages
#[embassy_executor::task]
pub async fn handle_message(devices: &'static mut I2CDevices<'static, I2c<'static, Blocking>>) -> ! {
    loop {
        let cmd = CHANNEL.receiver().receive().await;
        tracing::info!("Received I2CCommand: {:?}", cmd);

        match devices.execute_command(cmd) {
            Ok(Some((accel, gyro, temp))) => tracing::info!(?accel, ?gyro, ?temp, "IMU Data Read"),
            Ok(None) => tracing::info!("Command executed successfully"),
            Err(err) => tracing::error!(?err, "Failed to execute I2C command"),
        }
    }
}

/// Helper function to wait for network connection
async fn wait_for_network(stack: &Stack<WifiDevice<'static, WifiStaDevice>>) {
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
