#![allow(dead_code)]

use esp_hal::{i2c, i2c::master::I2c, Blocking};

pub struct Py260
{
    i2c: I2c<'static, Blocking>,
    address: u8,
}

impl Py260
{
    pub const ADDR: u8 = 0x1F;

    pub fn new(i2c: I2c<'static, Blocking>) -> Self
    {
        Self {
            i2c,
            address: Self::ADDR,
        }
    }

    fn read_reg(
        &mut self,
        reg: u16,
    ) -> Result<u8, i2c::master::Error>
    {
        let mut value = 0;
        self.i2c.write_read(
            self.address,
            &reg.to_be_bytes(),
            core::slice::from_mut(&mut value),
        )?;
        Ok(value)
    }

    fn write_reg(
        &mut self,
        reg: u16,
        value: u8,
    ) -> Result<(), i2c::master::Error>
    {
        let reg_bytes = reg.to_be_bytes();
        self.i2c
            .write(self.address, &[reg_bytes[0], reg_bytes[1], value])
    }

    pub fn reset(&mut self) -> Result<(), i2c::master::Error>
    {
        self.write_reg(CAMERA_RST_REG, 0x00)?;
        self.write_reg(CAMERA_RST_REG, 0x01)?;
        Ok(())
    }

    pub fn set_pixel_format(
        &mut self,
        pixel_format: PixelFormat,
    ) -> Result<(), i2c::master::Error>
    {
        let value = match pixel_format {
            PixelFormat::Jpeg => 0x01,
            PixelFormat::Rgb565 => 0x02,
            PixelFormat::Yuv422 => 0x03,
        };
        self.write_reg(PIXEL_FMT_REG, value)
    }

    pub fn set_resolution(
        &mut self,
        resolution: Resolution,
    ) -> Result<(), i2c::master::Error>
    {
        let value = match resolution {
            Resolution::Qvga => 0x01,
            Resolution::Vga => 0x02,
            Resolution::Hd => 0x03,
            Resolution::Uxga => 0x04,
            Resolution::Fhd => 0x05,
            Resolution::Max => 0x06,
            Resolution::M96x96 => 0x07,
            Resolution::Vga128x128 => 0x08,
            Resolution::Vga320x320 => 0x09,
        };
        self.write_reg(RESOLUTION_REG, value)
    }

    pub fn set_quality(
        &mut self,
        quality: u8,
    ) -> Result<(), i2c::master::Error>
    {
        self.write_reg(IMAGE_QUALITY_REG, quality)
    }

    pub fn set_hmirror(
        &mut self,
        enable: bool,
    ) -> Result<(), i2c::master::Error>
    {
        self.write_reg(IMAGE_MIRROR_REG, if enable { 0x01 } else { 0x00 })
    }

    pub fn set_vflip(
        &mut self,
        enable: bool,
    ) -> Result<(), i2c::master::Error>
    {
        self.write_reg(IMAGE_FLIP_REG, if enable { 0x01 } else { 0x00 })
    }

    pub fn set_brightness(
        &mut self,
        brightness: u8,
    ) -> Result<(), i2c::master::Error>
    {
        self.write_reg(BRIGHTNESS_REG, brightness.clamp(0, 8))
    }

    pub fn set_contrast(
        &mut self,
        contrast: u8,
    ) -> Result<(), i2c::master::Error>
    {
        self.write_reg(CONTRAST_REG, contrast.clamp(0, 6))
    }

    pub fn set_saturation(
        &mut self,
        saturation: u8,
    ) -> Result<(), i2c::master::Error>
    {
        self.write_reg(SATURATION_REG, saturation.clamp(0, 6))
    }

    pub fn read_sensor_id(&mut self) -> Result<u16, i2c::master::Error>
    {
        let h = self.read_reg(SENSOR_ID_HIGH)?;
        let l = self.read_reg(SENSOR_ID_LOW)?;
        Ok(u16::from_be_bytes([h, l]))
    }
}

pub enum PixelFormat
{
    Jpeg,
    Rgb565,
    Yuv422,
}

pub enum Resolution
{
    /// 320 x 240
    Qvga,

    /// 640 x 480
    Vga,

    /// 1280 x 720
    Hd,

    /// 1600 x 1200
    Uxga,

    /// 1920 x 1080
    Fhd,

    /// 2592 x 1944 (5MP)
    Max,

    /// 96 x 96
    M96x96,

    /// 128 x 128
    Vga128x128,

    /// 320 x 320
    Vga320x320,
}

const ID_BASE: u16 = 0x0000;
const SENSOR_BASE: u16 = 0x0100;
const SYS_CLK_BASE: u16 = 0x0200;
const BYPASS_BASE: u16 = 0xFFF0;

const SENSOR_ID_HIGH: u16 = ID_BASE + 0x00;
const SENSOR_ID_LOW: u16 = ID_BASE + 0x01;
const FIRMWARE_VER: u16 = ID_BASE + 0x02;

const CAMERA_RST_REG: u16 = SENSOR_BASE + 0x02;

const PIXEL_FMT_REG: u16 = SENSOR_BASE + 0x20;
const RESOLUTION_REG: u16 = SENSOR_BASE + 0x21;
const BRIGHTNESS_REG: u16 = SENSOR_BASE + 0x22;
const CONTRAST_REG: u16 = SENSOR_BASE + 0x23;
const SATURATION_REG: u16 = SENSOR_BASE + 0x24;
const EXP_COMPENSATE_REG: u16 = SENSOR_BASE + 0x25;
const AWB_MODE_REG: u16 = SENSOR_BASE + 0x26;
const SPECIAL_REG: u16 = SENSOR_BASE + 0x27;
const SHARPNESS_REG: u16 = SENSOR_BASE + 0x28;
const FOCUS_REG: u16 = SENSOR_BASE + 0x29;
const IMAGE_QUALITY_REG: u16 = SENSOR_BASE + 0x2A;
const IMAGE_FLIP_REG: u16 = SENSOR_BASE + 0x2B;
const IMAGE_MIRROR_REG: u16 = SENSOR_BASE + 0x2C;

const AGC_MODE_REG: u16 = SENSOR_BASE + 0x30;
const MANUAL_AGC_REG: u16 = SENSOR_BASE + 0x31;
const MANUAL_EXP_H_REG: u16 = SENSOR_BASE + 0x33;
const MANUAL_EXP_L_REG: u16 = SENSOR_BASE + 0x34;

const SYSTEM_CLK_DIV_REG: u16 = SYS_CLK_BASE + 0x00;
const SYSTEM_PLL_DIV_REG: u16 = SYS_CLK_BASE + 0x01;
