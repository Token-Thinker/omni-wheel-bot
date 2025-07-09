use clap::Parser;
use core::cell::RefCell;
use embassy_executor::{Executor, Spawner};
use embassy_net::{Config, Ipv4Address, Ipv4Cidr, Stack, StackResources};
use embassy_net_tuntap::TunTapDevice;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTrans};
use heapless::Vec;
use owb_core::utils::{SystemController, wss};
use owb_core::utils::controllers::{I2C_CHANNEL, LED_CHANNEL, LEDCommand, LedModule};
use rand_core::{OsRng, RngCore};
use static_cell::StaticCell;
use tracing::{info, error};
use tracing_subscriber;
use std::convert::Infallible;
use smart_leds_trait::{SmartLedsWrite, RGB8};

#[derive(Parser)]
#[clap(version = "1.0")]
struct Opts
{
    /// TAP device name
    #[clap(long, default_value = "tap0")]
    tap: String,
    /// use a static IP instead of DHCP
    #[clap(long)]
    static_ip: bool,
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<TunTapDevice>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
async fn i2c_task(mut ctrl: SystemController<I2cMock>) -> ! {
    ctrl.i2c_ch().await
}

#[embassy_executor::task]
async fn led_task(mut leds: LedModule<SerialLedDriver>) -> ! {
    loop {
        let cmd: LEDCommand = LED_CHANNEL.receiver().receive().await;
        if let Err(e) = leds.ex_command(cmd) {
            error!("LED command failed: {:?}", e);
        }
    }
}

#[embassy_executor::task]
async fn main_task(spawner: Spawner) {
    // I2C mock setup
    let mut expectations: Vec<I2cTrans, 16> = Vec::new();
    let mock = I2cMock::new(&expectations);
    static I2C_BUS: StaticCell<RefCell<I2cMock>> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(RefCell::new(mock));

    let sys_ctrl = SystemController::new(i2c_bus, None, None);
    spawner.spawn(i2c_task(sys_ctrl)).unwrap();

    // LED driver that logs to console
    struct SerialLedDriver;
    impl SmartLedsWrite for SerialLedDriver {
        type Color = RGB8;
        type Error = Infallible;

        fn write<I>(&mut self, iter: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Self::Color>,
        {
            for c in iter {
                info!("LED: {:?}", c);
            }
            Ok(())
        }
    }

    let leds = LedModule::new(SerialLedDriver);
    spawner.spawn(led_task(leds)).unwrap();

    // Parse CLI and initialize network
    let opts: Opts = Opts::parse();
    let device = TunTapDevice::new(&opts.tap).unwrap();
    let config = if opts.static_ip {
        Config::ipv4_static(embassy_net::StaticConfigV4 {
            address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 69, 2), 24),
            dns_servers: Vec::new(),
            gateway: Some(Ipv4Address::new(192, 168, 69, 1)),
        })
    } else {
        Config::dhcpv4(Default::default())
    };
    let mut seed_buf = [0; 8];
    OsRng.fill_bytes(&mut seed_buf);
    let seed = u64::from_le_bytes(seed_buf);

    static STACK: StaticCell<Stack<TunTapDevice>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let stack = &*STACK.init(Stack::new(
        device,
        config,
        RESOURCES.init(StackResources::<3>::new()),
        seed,
    ));
    spawner.spawn(net_task(stack)).unwrap();

    info!("Waiting for network link...");
    // TODO: wait for IP assignment if needed

    info!("Starting WebSocket server on port 8000");
    wss(0, 8000, stack, None).await;
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(main_task(spawner)).unwrap();
    });
}