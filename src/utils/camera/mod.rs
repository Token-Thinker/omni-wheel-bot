pub mod mega_ccm;

use core::ops::{Deref, DerefMut};
use esp_hal::dma::DmaRxStreamBuf;
#[cfg(feature = "esp32s3")]
use esp_hal::lcd_cam::cam::{Camera, CameraTransfer};

pub struct MyCamera<'a> {
    state: DriverState<'a>,
}

impl<'d> MyCamera<'d> {
    pub fn new(
        camera: Camera<'d>,
        buf: DmaRxStreamBuf,
    ) -> Self {
        Self {
            state: DriverState::Idle(camera, buf),
        }
    }

    pub fn receive<'a>(&'a mut self) -> MyCamTransfer<'a, 'd> {
        let state = core::mem::take(&mut self.state);
        let DriverState::Idle(camera, buf) = state else {
            unreachable!()
        };

        let transfer = camera.receive(buf).map_err(|e| e.0).unwrap();
        self.state = DriverState::Running(transfer);

        MyCamTransfer { driver: self }
    }
}

#[derive(Default)]
enum DriverState<'d> {
    Idle(Camera<'d>, DmaRxStreamBuf),
    Running(CameraTransfer<'d, DmaRxStreamBuf>),
    #[default]
    Borrowed,
}

pub struct MyCamTransfer<'a, 'd> {
    driver: &'a mut MyCamera<'d>,
}

impl<'a, 'd> Deref for MyCamTransfer<'a, 'd> {
    type Target = CameraTransfer<'d, DmaRxStreamBuf>;

    fn deref(&self) -> &Self::Target {
        match &self.driver.state {
            DriverState::Running(transfer) => transfer,
            _ => unreachable!(),
        }
    }
}

impl<'a, 'd> DerefMut for MyCamTransfer<'a, 'd> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match &mut self.driver.state {
            DriverState::Running(transfer) => transfer,
            _ => unreachable!(),
        }
    }
}

impl<'a, 'd> Drop for MyCamTransfer<'a, 'd> {
    fn drop(&mut self) {
        let state = core::mem::take(&mut self.driver.state);

        let DriverState::Running(transfer) = state else {
            unreachable!()
        };

        let (camera, buf) = transfer.stop();
        self.driver.state = DriverState::Idle(camera, buf);
    }
}
