#![no_std]
#![no_main]
extern crate alloc;

// Standard imports
use core::cell::RefCell;
use heapless::String;
// Embassy framework imports
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
// ESP-specific imports
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    i2c::master::{Config, I2c},
    rmt::{Channel, Rmt},
    rng::Rng,
    timer::timg::TimerGroup,
    Blocking,
    time::Rate,
};
use esp_wifi::{
    init,
    wifi::{
        ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState,
    },
    EspWifiController,
};
use log::LevelFilter;
//Internal Modules
use omni_wheel::{smart_led_buffer, utils::{
    connection::app_server,
    controllers::{I2CDevices, LedModule, I2C_CHANNEL, LED_CHANNEL},
    packages::smart_leds::SmartLedsAdapter,
    }
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
const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const BUF_SIZE: usize = 2 * 24 + 1;

// Static memory for I2C devices
static I2C_REF: static_cell::StaticCell<RefCell<I2c<'static, Blocking>>> =
    static_cell::StaticCell::new();
static DEVICES: static_cell::StaticCell<Option<I2CDevices<'static, I2c<'static, Blocking>>>> =
    static_cell::StaticCell::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {

    // General Configuration Block *****************************************************************
    esp_println::logger::init_logger(LevelFilter::Trace);
    tracing::info!("Logger initialized");
    
    let esp_config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(esp_config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    
    let mut rng = Rng::new(peripherals.RNG);
    // End of General Configuration Block **********************************************************
    
    //Peripherals Configuration ********************************************************************
    let sda_per = peripherals.GPIO21;
    let scl_per = peripherals.GPIO22;

    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_sda(sda_per)
        .with_scl(scl_per);
    let i2c_ref = I2C_REF.init(RefCell::new(i2c));

    let devices_init = I2CDevices::new(i2c_ref, 0.148, 0.195)
        .map(|mut dev| {
            if let Err(e) = dev.configure_pwm() {
                tracing::warn!("Additional PWM configuration failed: {:?}", e);
            }
            dev.init_imu_data();
            dev
        })
        .or_else(|e| {
            tracing::warn!("Skipping I2C/IMU usage: {:?}", e);
            I2CDevices::scan(&i2c_ref);
            Err(e)
        })
        .ok();

    let devices = DEVICES.init(devices_init);

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32h2")] {
            let freq = Rate::from_mhz(32);
        } else {
            let freq = Rate::from_mhz(80);
        }
    }


    let rmt = Rmt::new(peripherals.RMT, freq).unwrap();
    let rmt_buffer = smart_led_buffer!(2);
    let adapter = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO12, rmt_buffer);
    let leds = LedModule::new(adapter);


    
    // Wi-Fi Configuration Block *******************************************************************
    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );

    let(controller, interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

    let wifi_sta_device = interfaces.sta;
    let dhcpv_config = {
        let mut dhcpv = embassy_net::DhcpConfig::default();
        dhcpv.server_port = 67;
        dhcpv.client_port = 68;
        dhcpv.hostname = Some(String::try_from("controller").unwrap());
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
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );
    // End of Wi-Fi Configuration Block ************************************************************

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(sta_runner)).ok();
    
    wait_for_network(&sta_stack).await;
    
    spawner.spawn(handle_i2c_message(devices)).unwrap();
    spawner.spawn(handle_leds_message(leds)).unwrap();
    
    app_server(0, 80, sta_stack, None).await;
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
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

/// Task to handle I2C messages
#[embassy_executor::task]
pub async fn handle_i2c_message(
    devices: &'static mut Option<I2CDevices<'static, I2c<'static, Blocking>>>
) -> ! {
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
) -> ! {
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
async fn wait_for_network(stack: &Stack<'static>) {
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
