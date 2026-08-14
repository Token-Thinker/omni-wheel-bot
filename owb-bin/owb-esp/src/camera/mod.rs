//! ESP32-S3 camera bring-up helpers adapted from the earlier camera branch.

mod ov2640;

use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicU16, AtomicU32, Ordering},
};

use esp_hal::{
    dma::DmaRxStreamBuf,
    dma_rx_stream_buffer,
    gpio::{AnyPin, Level, Output, OutputConfig},
    i2c::master::I2c,
    lcd_cam::{
        cam::{Camera, Config as CamConfig},
        LcdCam,
    },
    peripherals::{DMA_CH0, LCD_CAM},
    time::Rate,
    Blocking,
};
use owb_core::ext::prelude::{Duration, Timer};
use picoserve::response::{
    chunked::{ChunkWriter, ChunkedResponse, Chunks, ChunksWritten},
    StatusCode,
};

use crate::profile::BoardProfile;

#[derive(Debug, Clone, Copy)]
pub enum CameraError {
    Config,
    Sccb,
}

struct CameraState {
    camera: Option<Camera<'static>>,
    dma_buf: Option<DmaRxStreamBuf>,
    _pwdn: Option<Output<'static>>,
    _reset: Option<Output<'static>>,
}

struct CameraSlot(UnsafeCell<Option<CameraState>>);

// Safety: access is serialized by initialization ordering (`CAMERA_READY`) and
// single-stream gating (`CAMERA_STREAM_ACTIVE`).
unsafe impl Sync for CameraSlot {}

impl CameraSlot {
    const fn new() -> Self { Self(UnsafeCell::new(None)) }

    fn replace(
        &self,
        state: CameraState,
    ) {
        unsafe {
            *self.0.get() = Some(state);
        }
    }

    fn with_mut<R>(
        &self,
        f: impl FnOnce(&mut Option<CameraState>) -> R,
    ) -> R {
        unsafe { f(&mut *self.0.get()) }
    }
}

static CAMERA_STATE: CameraSlot = CameraSlot::new();
static CAMERA_READY: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
static CAMERA_STREAM_ACTIVE: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);
static CAMERA_DEVICE_ENABLED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(true);
static CAMERA_FRAME_INTERVAL_MS: AtomicU16 = AtomicU16::new(100);
static SNAP_REQUESTED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
// Packed (width << 16) | height set during camera init.
static CAMERA_DIMS: AtomicU32 = AtomicU32::new((160 << 16) | 120);

const MJPEG_CONTENT_TYPE: &str = "multipart/x-mixed-replace; boundary=frame";
const MJPEG_PREAMBLE: &[u8] = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ";

pub async fn init(
    sccb: &mut I2c<'_, Blocking>,
    lcd_cam: LCD_CAM<'static>,
    dma: DMA_CH0<'static>,
    profile: &BoardProfile,
) -> Result<(), CameraError> {
    let camera = build_camera(lcd_cam, dma, profile)?;
    let power_pins = configure_power_pins(profile);
    ov2640_init(sccb, profile.width, profile.height, profile.jpeg_quality_qs).await?;

    const DMA_BUFFER_LEN: usize = 40 * 1024;
    const DMA_CHUNK_LEN: usize = 2048;
    let dma_buf = dma_rx_stream_buffer!(DMA_BUFFER_LEN, DMA_CHUNK_LEN);

    CAMERA_STATE.replace(CameraState {
        camera: Some(camera),
        dma_buf: Some(dma_buf),
        _pwdn: power_pins.pwdn,
        _reset: power_pins.reset,
    });

    CAMERA_DIMS.store(
        ((profile.width as u32) << 16) | (profile.height as u32),
        Ordering::Release,
    );
    CAMERA_FRAME_INTERVAL_MS.store(profile.frame_interval_ms, Ordering::Release);
    CAMERA_DEVICE_ENABLED.store(true, core::sync::atomic::Ordering::Release);
    CAMERA_READY.store(true, core::sync::atomic::Ordering::Release);
    Ok(())
}

pub fn toggle_device_enabled() -> bool {
    let next = !CAMERA_DEVICE_ENABLED.load(core::sync::atomic::Ordering::Acquire);
    CAMERA_DEVICE_ENABLED.store(next, core::sync::atomic::Ordering::Release);
    next
}

fn build_camera(
    lcd_cam: LCD_CAM<'static>,
    dma: DMA_CH0<'static>,
    profile: &BoardProfile,
) -> Result<Camera<'static>, CameraError> {
    let config = CamConfig::default().with_frequency(Rate::from_mhz(20));
    let lcd_cam = LcdCam::new(lcd_cam);
    let camera = Camera::new(lcd_cam.cam, dma, config)
        .map_err(|_| CameraError::Config)?
        .with_master_clock(unsafe { AnyPin::steal(profile.camera.xclk) })
        .with_pixel_clock(unsafe { AnyPin::steal(profile.camera.pclk) })
        .with_vsync(unsafe { AnyPin::steal(profile.camera.vsync) })
        .with_h_enable(unsafe { AnyPin::steal(profile.camera.href) })
        .with_data0(unsafe { AnyPin::steal(profile.camera.d0) })
        .with_data1(unsafe { AnyPin::steal(profile.camera.d1) })
        .with_data2(unsafe { AnyPin::steal(profile.camera.d2) })
        .with_data3(unsafe { AnyPin::steal(profile.camera.d3) })
        .with_data4(unsafe { AnyPin::steal(profile.camera.d4) })
        .with_data5(unsafe { AnyPin::steal(profile.camera.d5) })
        .with_data6(unsafe { AnyPin::steal(profile.camera.d6) })
        .with_data7(unsafe { AnyPin::steal(profile.camera.d7) });

    Ok(camera)
}

struct CameraPowerPins {
    pwdn: Option<Output<'static>>,
    reset: Option<Output<'static>>,
}

fn configure_power_pins(profile: &BoardProfile) -> CameraPowerPins {
    let pwdn = profile.camera.pwdn.map(|pin| {
        let mut pin = Output::new(
            unsafe { AnyPin::steal(pin) },
            Level::Low,
            OutputConfig::default(),
        );
        pin.set_low();
        pin
    });

    let reset = profile.camera.reset.map(|pin| {
        let mut pin = Output::new(
            unsafe { AnyPin::steal(pin) },
            Level::High,
            OutputConfig::default(),
        );
        pin.set_high();
        pin
    });

    CameraPowerPins { pwdn, reset }
}

async fn ov2640_init(
    sccb: &mut I2c<'_, Blocking>,
    width: u16,
    height: u16,
    _jpeg_quality_qs: u8,
) -> Result<(), CameraError> {
    write_reg(sccb, ov2640::BANK_SEL, ov2640::BANK_SENSOR)?;
    write_reg(sccb, ov2640::COM7, 0x80)?;
    Timer::after(Duration::from_millis(10)).await;

    write_regs(sccb, ov2640::OV2640_SETTINGS_CIF)?;
    Timer::after(Duration::from_millis(10)).await;

    set_window(sccb, width, height).await?;
    // OV2640_SETTINGS_TO_CIF (inside set_window) leaves RESET = RESET_DVP without
    // clearing it; JPEG3 init used to clear it via its trailing (RESET, 0x00).
    // In YUV mode we skip JPEG3, so clear the DVP reset manually here.
    write_reg_in_bank(sccb, ov2640::BANK_DSP, ov2640::RESET, 0x00)?;
    Timer::after(Duration::from_millis(10)).await;
    // IMAGE_MODE = 0x00 (set by OV2640_SETTINGS_CIF) → YUV422 YUYV output.

    Ok(())
}

async fn set_window(
    sccb: &mut I2c<'_, Blocking>,
    width: u16,
    height: u16,
) -> Result<(), CameraError> {
    let mut max_x: u16 = 1600;
    let mut max_y: u16 = 1200;
    let mut offset_x: u16 = 0;
    let mut offset_y: u16 = 0;

    max_x /= 4;
    max_y /= 4;
    offset_x /= 4;
    offset_y /= 4;

    if max_y > 296 {
        max_y = 296;
    }

    set_window_inner(sccb, offset_x, offset_y, max_x, max_y, width, height).await
}

async fn set_window_inner(
    sccb: &mut I2c<'_, Blocking>,
    offset_x: u16,
    offset_y: u16,
    mut max_x: u16,
    mut max_y: u16,
    mut width: u16,
    mut height: u16,
) -> Result<(), CameraError> {
    max_x /= 4;
    max_y /= 4;
    width /= 4;
    height /= 4;

    let win_regs: [(u8, u8); 11] = [
        (ov2640::BANK_SEL, ov2640::BANK_DSP),
        (ov2640::HSIZE, (max_x & 0xff) as u8),
        (ov2640::VSIZE, (max_y & 0xff) as u8),
        (ov2640::XOFFL, (offset_x & 0xff) as u8),
        (ov2640::YOFFL, (offset_y & 0xff) as u8),
        (
            ov2640::VHYX,
            ((max_y >> 1) as u8 & 0x80)
                | ((offset_y >> 4) as u8 & 0x70)
                | ((max_x >> 5) as u8 & 0x08)
                | ((offset_x >> 8) as u8 & 0x07),
        ),
        (ov2640::TEST, ((max_x >> 2) as u8) & 0x80),
        (ov2640::ZMOW, (width & 0xff) as u8),
        (ov2640::ZMOH, (height & 0xff) as u8),
        (
            ov2640::ZMHH,
            (((height >> 6) as u8) & 0x04) | (((width >> 8) as u8) & 0x03),
        ),
        (0, 0),
    ];

    let clk_div: u8 = 0;
    let clk_2x: u8 = 0;
    let pclk_div: u8 = 8;
    let pclk_auto: u8 = 0;
    let clk = (clk_2x << 7) | (clk_div & 0x3f);
    let pclk = (pclk_auto << 7) | (pclk_div & 0x7f);

    write_reg_in_bank(
        sccb,
        ov2640::BANK_DSP,
        ov2640::R_BYPASS,
        ov2640::R_BYPASS_DSP_BYPAS,
    )?;
    write_regs(sccb, ov2640::OV2640_SETTINGS_TO_CIF)?;
    write_regs(sccb, &win_regs)?;
    write_reg_in_bank(sccb, ov2640::BANK_SENSOR, ov2640::CLKRC, clk)?;
    write_reg_in_bank(sccb, ov2640::BANK_DSP, ov2640::R_DVP_SP, pclk)?;
    write_reg_in_bank(
        sccb,
        ov2640::BANK_DSP,
        ov2640::R_BYPASS,
        ov2640::R_BYPASS_DSP_EN,
    )?;
    Timer::after(Duration::from_millis(10)).await;

    Ok(())
}

fn write_regs(
    sccb: &mut I2c<'_, Blocking>,
    regs: &[(u8, u8)],
) -> Result<(), CameraError> {
    for &(reg, val) in regs {
        if reg == 0 && val == 0 {
            break;
        }
        write_reg(sccb, reg, val)?;
    }
    Ok(())
}

fn write_reg(
    sccb: &mut I2c<'_, Blocking>,
    reg: u8,
    val: u8,
) -> Result<(), CameraError> {
    sccb.write(ov2640::SCCB_ADDR, &[reg, val])
        .map_err(|_| CameraError::Sccb)
}

fn write_reg_in_bank(
    sccb: &mut I2c<'_, Blocking>,
    bank: u8,
    reg: u8,
    val: u8,
) -> Result<(), CameraError> {
    write_reg(sccb, ov2640::BANK_SEL, bank)?;
    write_reg(sccb, reg, val)
}

fn find_jpeg(buf: &[u8]) -> Option<(usize, usize)> {
    let mut start = None;
    for i in 0..buf.len().saturating_sub(1) {
        if buf[i] == 0xff && buf[i + 1] == 0xd8 {
            start = Some(i);
            break;
        }
    }

    let start = start?;
    for i in (start + 2)..buf.len().saturating_sub(1) {
        if buf[i] == 0xff && buf[i + 1] == 0xd9 {
            return Some((start, i + 2));
        }
    }

    None
}

fn write_content_length(
    mut len: usize,
    out: &mut [u8],
) -> usize {
    let mut tmp = [0u8; 10];
    let mut i = 0;

    if len == 0 {
        tmp[0] = b'0';
        i = 1;
    }
    else {
        while len > 0 && i < tmp.len() {
            tmp[i] = b'0' + (len % 10) as u8;
            len /= 10;
            i += 1;
        }
    }

    for j in 0..i {
        out[j] = tmp[i - 1 - j];
    }

    i
}

impl CameraState {
    fn next_frame_ptr(&mut self) -> Option<(*const u8, usize)> {
        let camera = self.camera.take()?;
        let dma_buf = self.dma_buf.take()?;

        let transfer = match camera.receive(dma_buf) {
            Ok(transfer) => transfer,
            Err((_err, camera, dma_buf)) => {
                self.camera = Some(camera);
                self.dma_buf = Some(dma_buf);
                return None;
            }
        };

        let (result, camera, dma_buf) = transfer.wait();
        self.camera = Some(camera);

        if result.is_err() {
            self.dma_buf = Some(dma_buf);
            return None;
        }

        let (descs, buffer) = dma_buf.split();
        let found = find_jpeg(buffer);
        let frame_ptr = found.map(|(start, _end)| unsafe { buffer.as_ptr().add(start) });
        let frame_len = found.map(|(start, end)| end - start);
        let rebuilt = DmaRxStreamBuf::new(descs, buffer).ok()?;
        self.dma_buf = Some(rebuilt);

        match (frame_ptr, frame_len) {
            (Some(ptr), Some(len)) => Some((ptr, len)),
            _ => None,
        }
    }
}

struct StreamGuard;

impl StreamGuard {
    fn acquire() -> Option<Self> {
        CAMERA_STREAM_ACTIVE
            .compare_exchange(
                false,
                true,
                core::sync::atomic::Ordering::AcqRel,
                core::sync::atomic::Ordering::Acquire,
            )
            .ok()
            .map(|_| StreamGuard)
    }
}

impl Drop for StreamGuard {
    fn drop(&mut self) { CAMERA_STREAM_ACTIVE.store(false, core::sync::atomic::Ordering::Release); }
}

pub(crate) struct MjpegStream {
    _guard: StreamGuard,
}

impl Chunks for MjpegStream {
    fn content_type(&self) -> &'static str { MJPEG_CONTENT_TYPE }

    async fn write_chunks<W: picoserve::io::Write>(
        self,
        mut chunk_writer: ChunkWriter<W>,
    ) -> Result<ChunksWritten, W::Error> {
        loop {
            if !CAMERA_DEVICE_ENABLED.load(core::sync::atomic::Ordering::Acquire) {
                return chunk_writer.finalize().await;
            }

            let frame =
                CAMERA_STATE.with_mut(|state| state.as_mut().and_then(CameraState::next_frame_ptr));

            if let Some((ptr, len)) = frame {
                let jpeg = unsafe { core::slice::from_raw_parts(ptr, len) };
                let mut header = [0u8; 96];
                let mut idx = 0;
                header[..MJPEG_PREAMBLE.len()].copy_from_slice(MJPEG_PREAMBLE);
                idx += MJPEG_PREAMBLE.len();
                let len_len = write_content_length(len, &mut header[idx..]);
                idx += len_len;
                header[idx..idx + 4].copy_from_slice(b"\r\n\r\n");
                idx += 4;

                chunk_writer.write_chunk(&header[..idx]).await?;
                chunk_writer.write_chunk(jpeg).await?;
                chunk_writer.write_chunk(b"\r\n").await?;

                let frame_interval_ms = CAMERA_FRAME_INTERVAL_MS.load(Ordering::Acquire);
                if frame_interval_ms > 0 {
                    Timer::after(Duration::from_millis(frame_interval_ms as u64)).await;
                }
            }
            else {
                Timer::after(Duration::from_millis(10)).await;
            }
        }
    }
}

/// Signal the snap task to capture one frame and emit it as base64 over USB serial.
/// Non-blocking: the actual DMA capture happens in `camera_snap_task` on the NET executor.
pub fn snap() {
    if !CAMERA_READY.load(core::sync::atomic::Ordering::Acquire) {
        esp_println::println!("[SNAP] error: camera not ready");
        return;
    }
    SNAP_REQUESTED.store(true, core::sync::atomic::Ordering::Release);
    esp_println::println!("[SNAP] requested");
}

fn base64_emit(data: &[u8]) {
    const ALPHA: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut line_buf = [0u8; 77];
    for chunk in data.chunks(57) {
        let mut out = 0usize;
        let mut i = 0usize;
        while i < chunk.len() {
            let b0 = chunk[i];
            let b1 = if i + 1 < chunk.len() { chunk[i + 1] } else { 0 };
            let b2 = if i + 2 < chunk.len() { chunk[i + 2] } else { 0 };
            line_buf[out] = ALPHA[(b0 >> 2) as usize];
            line_buf[out + 1] = ALPHA[((b0 & 0x03) << 4 | b1 >> 4) as usize];
            line_buf[out + 2] = if i + 1 < chunk.len() {
                ALPHA[((b1 & 0x0f) << 2 | b2 >> 6) as usize]
            }
            else {
                b'='
            };
            line_buf[out + 3] = if i + 2 < chunk.len() {
                ALPHA[(b2 & 0x3f) as usize]
            }
            else {
                b'='
            };
            out += 4;
            i += 3;
        }
        line_buf[out] = b'\n';
        esp_println::print!("{}", unsafe {
            core::str::from_utf8_unchecked(&line_buf[..out + 1])
        });
    }
}

// Emits a YUV422 (YUYV) frame as base64.  Width/height let the host decode it.
// Format: [SNAP_RAW] begin <w> <h> <bytes>\n<base64 lines>\n[SNAP_RAW] end
fn emit_snap_raw(
    buf: &[u8],
    width: u16,
    height: u16,
) {
    let byte_count = (width as usize) * (height as usize) * 2;
    let data = &buf[..byte_count.min(buf.len())];
    esp_println::println!("[SNAP_RAW] begin {} {} {}", width, height, data.len());
    base64_emit(data);
    esp_println::println!("[SNAP_RAW] end");
}

/// Run one camera DMA transfer, blocking until the buffer is full.
/// Returns Ok((camera, dma_buf)) on success.
fn run_capture_blocking(
    camera: esp_hal::lcd_cam::cam::Camera<'static>,
    dma_buf: DmaRxStreamBuf,
) -> Result<(esp_hal::lcd_cam::cam::Camera<'static>, DmaRxStreamBuf), &'static str> {
    match camera.receive(dma_buf) {
        Err((_, camera, dma_buf)) => {
            CAMERA_STATE.with_mut(|s| {
                if let Some(st) = s.as_mut() {
                    st.camera = Some(camera);
                    st.dma_buf = Some(dma_buf);
                }
            });
            Err("DMA start failed")
        }
        Ok(transfer) => {
            let (result, camera, dma_buf) = transfer.wait();
            if result.is_err() {
                CAMERA_STATE.with_mut(|s| {
                    if let Some(st) = s.as_mut() {
                        st.camera = Some(camera);
                        st.dma_buf = Some(dma_buf);
                    }
                });
                return Err("DMA error");
            }
            Ok((camera, dma_buf))
        }
    }
}

/// Embassy task that runs on core 0.  Captures one YUV422 frame on each snap
/// request and emits it base64-encoded over USB serial as [SNAP_RAW].
pub async fn snap_task() -> ! {
    loop {
        if !SNAP_REQUESTED.load(core::sync::atomic::Ordering::Acquire) {
            Timer::after(Duration::from_millis(50)).await;
            continue;
        }
        SNAP_REQUESTED.store(false, core::sync::atomic::Ordering::Release);

        let taken = CAMERA_STATE.with_mut(|s| {
            s.as_mut().and_then(|state| {
                let cam = state.camera.take();
                let buf = state.dma_buf.take();
                cam.zip(buf)
            })
        });

        let Some((camera, dma_buf)) = taken
        else {
            esp_println::println!("[SNAP] error: camera state not available");
            Timer::after(Duration::from_millis(50)).await;
            continue;
        };

        // Single-pass YUV422 capture.  OV2640 in YUV mode uses VSYNC for full
        // frame boundaries, so one DMA transfer = one complete frame.
        match run_capture_blocking(camera, dma_buf) {
            Err(msg) => {
                esp_println::println!("[SNAP] capture error: {}", msg);
            }
            Ok((camera, dma_buf)) => {
                CAMERA_STATE.with_mut(|s| {
                    if let Some(st) = s.as_mut() {
                        st.camera = Some(camera);
                    }
                });
                let (descs, buffer) = dma_buf.split();
                let dims = CAMERA_DIMS.load(Ordering::Acquire);
                let (w, h) = ((dims >> 16) as u16, (dims & 0xffff) as u16);
                emit_snap_raw(buffer, w, h);
                if let Ok(rebuilt) = DmaRxStreamBuf::new(descs, buffer) {
                    CAMERA_STATE.with_mut(|s| {
                        if let Some(st) = s.as_mut() {
                            st.dma_buf = Some(rebuilt);
                        }
                    });
                }
            }
        }

        Timer::after(Duration::from_millis(50)).await;
    }
}

pub async fn stream() -> Result<ChunkedResponse<MjpegStream>, (StatusCode, &'static str)> {
    if !CAMERA_READY.load(core::sync::atomic::Ordering::Acquire) {
        return Err((StatusCode::SERVICE_UNAVAILABLE, "camera not initialized"));
    }
    if !CAMERA_DEVICE_ENABLED.load(core::sync::atomic::Ordering::Acquire) {
        return Err((StatusCode::SERVICE_UNAVAILABLE, "camera disabled"));
    }

    let _guard = StreamGuard::acquire().ok_or((StatusCode::TOO_MANY_REQUESTS, "camera busy"))?;

    Ok(ChunkedResponse::new(MjpegStream { _guard }))
}
