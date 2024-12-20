#![no_std]
#![no_main]

mod utils;
use utils::connection::{run as websocket_server};
use utils::controllers::{I2CDevices, CHANNEL};

use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Ipv4Address, Stack, StackResources};
use embassy_time::{Duration, Timer};

use esp_alloc as _;
use esp_hal::{prelude::*, rng::Rng, timer::timg::TimerGroup, i2c::master::{Config, I2c}, Blocking};
use esp_wifi::{
    init,
    wifi::{
        ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice,
        WifiState,
    },
    EspWifiController,
};

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

//const SSID: &str = option_env!("SSID").unwrap_or("Guest");
//const PASSWORD: &str = option_env!("PASSWORD").unwrap_or("password123");

const SSID: &str = "Test";
const PASSWORD: &str = "Test";

static I2C_REF: static_cell::StaticCell<RefCell<I2c<'static, Blocking>>> = static_cell::StaticCell::new();
static DEVICES: static_cell::StaticCell<I2CDevices<'static, I2c<'static, Blocking>>> = static_cell::StaticCell::new();


#[panic_handler]
pub fn panic(_info: &core::panic::PanicInfo) -> ! { loop {} }

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

    let i2c_ref = I2C_REF.init(RefCell::new(i2c));
    let devices = DEVICES.init(
        I2CDevices::new(i2c_ref, 0.148, 0.195).expect("Failed to initialize devices")
    );
    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let init = &*mk_static!(
        EspWifiController<'static>,
        init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap()
    );

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();

    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

    let config = embassy_net::Config::dhcpv4(Default::default());

    let seed = 1234; // very random, very secure seed

    // Init network stack
    let stack = &*mk_static!(
        Stack<WifiDevice<'_, WifiStaDevice>>,
        Stack::new(
            wifi_interface,
            config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed
        )
    );

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(&stack)).ok();

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

    spawner.spawn(handle_message(devices)).unwrap();

    // Run the WebSocket comms
    websocket_server(
        0,    // ID for the WebSocket comms instance
        8000, // Port number
        stack, None,
    )
        .await;


}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    tracing::info!("start connection task");
    tracing::info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
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
            tracing::info!("Starting wifi");
            controller.start_async().await.unwrap();
            tracing::info!("Wifi started!");
        }
        tracing::info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => tracing::info!("Wifi connected!"),
            Err(e) => {
                tracing::info!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}

#[embassy_executor::task]
pub async fn handle_message(
    devices: &'static mut I2CDevices<'static, I2c<'static, Blocking>>,
) -> !
{
    loop {
        // Wait for next command from the channel
        let cmd = CHANNEL.receiver().receive().await;
        tracing::info!("Received I2CCommand: {:?}", cmd);

        // Execute the command on I2CDevices
        match devices.execute_command(cmd) {
            Ok(Some((accel, gyro, temp))) => {
                // The command was ReadIMU, returning sensor data
                tracing::info!(?accel, ?gyro, ?temp, "IMU Data Read");
            }
            Ok(None) => {
                // Other commands (Enable, Disable, T, Y, O) return no data
                tracing::info!("Command executed successfully");
            }
            Err(err) => {
                tracing::error!(?err, "Failed to execute I2C command");
            }
        }
    }
}

