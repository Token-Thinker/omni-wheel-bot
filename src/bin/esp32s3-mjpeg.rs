#![feature(impl_trait_in_assoc_type)]
#![feature(allocator_api)]
#![no_std]
#![no_main]

// https://github.com/espressif/esp32-camera/pull/701/files#diff-a6a9b97c4bf45ceae4a2e05e9032fe34fae7360a35457bc8158c7c816f092ef5R296-R297
// https://docs.m5stack.com/en/unit/Unit-CAMS3%205MP
// https://github.com/m5stack/UnitCamS3-UserDemo/blob/unitcams3-5mp/platforms/unitcam_s3_5mp/components/esp32-camera/sensors/mega_ccm.c

use alloc::vec::Vec;
use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_net::{Stack, StackResources};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma_rx_stream_buffer,
    gpio::{Level, Output},
    i2c,
    i2c::master::{BusTimeout, I2c},
    lcd_cam::{
        cam,
        cam::{Camera, RxEightBits},
        LcdCam,
    },
    peripherals::Peripherals,
    psram::{
        psram_raw_parts,
        FlashFreq,
        PsramConfig,
        PsramSize,
        SpiRamFreq,
        SpiTimingConfigCoreClock,
    },
    rng::Rng,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_hal_embassy::main;
use esp_println::println;
use esp_wifi::{
    config::PowerSaveMode,
    wifi::{ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState},
    EspWifiController,
};
use log::LevelFilter;
use omni_wheel::utils::camera::{
    mega_ccm::{PixelFormat, Py260, Resolution},
    MyCamera,
};
use picoserve::{
    response::chunked::{ChunkWriter, ChunkedResponse, Chunks, ChunksWritten},
    routing::get,
};
use static_cell::{ConstStaticCell, StaticCell};

extern crate alloc;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

static PSRAM_HEAP: embedded_alloc::LlffHeap = embedded_alloc::LlffHeap::empty();

#[main]
async fn main(spawner: Spawner)
{
    let peripherals: Peripherals = esp_hal::init(
        esp_hal::Config::default()
            .with_cpu_clock(CpuClock::max())
            .with_psram(PsramConfig {
                size: PsramSize::AutoDetect,
                core_clock: SpiTimingConfigCoreClock::SpiTimingConfigCoreClock240m,
                flash_frequency: FlashFreq::FlashFreq80m,
                ram_frequency: SpiRamFreq::Freq80m,
            }),
    );

    let (start, size) = psram_raw_parts(&peripherals.PSRAM);
    unsafe { PSRAM_HEAP.init(start as _, size) };
    println!("PSRAM size = {}", size);

    esp_alloc::heap_allocator!(#[link_section = ".dram2_uninit"] size: 64000);
    esp_alloc::heap_allocator!(size: 150 * 1024);

    esp_println::logger::init_logger(LevelFilter::Info);

    let timer_group0 = TimerGroup::new(peripherals.TIMG0);
    let timer_group1 = TimerGroup::new(peripherals.TIMG1);
    let mut rng = Rng::new(peripherals.RNG);
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    let i2c_config = i2c::master::Config::default()
        .with_frequency(Rate::from_khz(100))
        .with_timeout(BusTimeout::Disabled);
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO17)
        .with_scl(peripherals.GPIO41);

    let camera = Camera::new(
        lcd_cam.cam,
        peripherals.DMA_CH0,
        RxEightBits::new(
            peripherals.GPIO6,
            peripherals.GPIO15,
            peripherals.GPIO16,
            peripherals.GPIO7,
            peripherals.GPIO5,
            peripherals.GPIO10,
            peripherals.GPIO4,
            peripherals.GPIO13,
        ),
        cam::Config::default().with_frequency(Rate::from_mhz(20)),
    )
    .unwrap()
    .with_master_clock(peripherals.GPIO11)
    .with_pixel_clock(peripherals.GPIO12)
    .with_ctrl_pins(peripherals.GPIO42, peripherals.GPIO18);

    let mut cam_reset = Output::new(peripherals.GPIO21, Level::Low, Default::default());
    esp_hal::delay::Delay::new().delay_millis(1000);
    cam_reset.set_high();

    let py260 = Py260::new(i2c);

    esp_hal_embassy::init(timer_group1.timer0);

    // region --------------------- WiFi setup ----------------------------

    let seed = rng.random() as u64;

    let init = esp_wifi::init(timer_group0.timer0, rng, peripherals.RADIO_CLK).unwrap();

    let init = &*{
        static WIFI_INIT: StaticCell<EspWifiController<'static>> = StaticCell::new();
        WIFI_INIT.init(init)
    };

    let (controller, wifi_interfaces) = esp_wifi::wifi::new(init, peripherals.WIFI).unwrap();

    let stack_resources = {
        static STACK_RESOURCES: ConstStaticCell<StackResources<3>> =
            ConstStaticCell::new(StackResources::new());
        STACK_RESOURCES.take()
    };
    let (stack, runner) = embassy_net::new(
        wifi_interfaces.sta,
        embassy_net::Config::dhcpv4(Default::default()),
        stack_resources,
        seed,
    );
    spawner.spawn(connection(controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();

    stack.wait_link_up().await;

    println!("Waiting to get IP address...");

    loop {
        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
    }

    // endregion ------------------------------------------------

    run_server(stack, camera, py260).await;
}

async fn run_server(
    stack: Stack<'static>,
    camera: Camera<'static>,
    py260: Py260,
)
{
    let dma_buf = dma_rx_stream_buffer!(64_000, 2000);

    let camera = RefCell::new(MyCamera::new(camera, dma_buf));
    let py260 = RefCell::new(py260);

    let config = picoserve::Config::new(picoserve::Timeouts {
        start_read_request: Some(Duration::from_secs(5)),
        persistent_start_read_request: None,
        read_request: Some(Duration::from_secs(1)),
        write: Some(Duration::from_secs(1)),
    });

    let stream_router = picoserve::Router::new().route(
        "/stream",
        get(|| async {
            let mut py260 = py260.borrow_mut();
            py260.reset().unwrap();
            Timer::after(Duration::from_secs(1)).await;
            py260.set_pixel_format(PixelFormat::Jpeg).unwrap();
            py260.set_resolution(Resolution::Vga).unwrap();
            py260.set_quality(16).unwrap();
            // py260.set_vflip(true).unwrap();
            // py260.set_hmirror(true).unwrap();

            ChunkedResponse::new(ImageStream { camera: &camera })
        }),
    );

    let mut tx_buf = Vec::new_in(&PSRAM_HEAP);
    tx_buf.resize(10 * 1024, 0);

    picoserve::listen_and_serve(
        0,
        &stream_router,
        &config,
        stack,
        80,
        &mut [0; 1024],
        // &mut [0; 1024 * 40],
        &mut tx_buf,
        &mut [0; 2048],
    )
    .await;
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>)
{
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.capabilities());

    controller.set_power_saving(PowerSaveMode::None).unwrap();

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        ..Default::default()
    });

    loop {
        if let WifiState::StaConnected = esp_wifi::wifi::wifi_state() {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(5000)).await
        }

        if !matches!(controller.is_started(), Ok(true)) {
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start_async().await.unwrap();
            println!("Wifi started!");
        }
        println!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut stack: embassy_net::Runner<'static, WifiDevice<'static>>)
{
    stack.run().await
}

struct ImageStream<'ch>
{
    camera: &'ch RefCell<MyCamera<'static>>,
}

impl<'ch> Chunks for ImageStream<'ch>
{
    fn content_type(&self) -> &'static str
    {
        "multipart/x-mixed-replace;boundary=123456789000000000000987654321"
    }

    async fn write_chunks<W: Write>(
        self,
        mut chunk_writer: ChunkWriter<W>,
    ) -> Result<ChunksWritten, W::Error>
    {
        let mut camera = self.camera.borrow_mut();
        let mut transfer = camera.receive();

        loop {
            chunk_writer
                .write_chunk(
                    b"\r\n--123456789000000000000987654321\r\nContent-Type: image/jpeg\r\n\r\n",
                )
                .await?;

            if transfer.is_done() {
                drop(transfer);
                transfer = camera.receive();
            }

            loop {
                let (data, ends_with_eof) = transfer.peek_until_eof();

                if data.is_empty() {
                    if transfer.is_done() {
                        println!("Too slow!");
                        break;
                    }

                    // TODO: Use interrupt.
                    Timer::after_micros(1000).await;
                    continue;
                }

                chunk_writer.write_chunk(&data).await?;
                let available = data.len();
                transfer.consume(available);
                if ends_with_eof {
                    break;
                }
            }
        }
    }
}
