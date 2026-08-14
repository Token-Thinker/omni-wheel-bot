use crate::profile::BoardProfile;

#[derive(Clone, Copy, Debug)]
pub struct BoardCapabilities {
    pub has_camera: bool,
    pub has_motion_driver: bool,
    pub has_status_led: bool,
    pub has_addressable_led: bool,
    pub has_tca9555: bool,
    pub tca9555_addr: Option<u8>,
    pub has_lis2dh12: bool,
    pub has_hdc2010: bool,
    pub has_max17048: bool,
}

impl BoardCapabilities {
    pub fn for_profile(profile: &BoardProfile) -> Self {
        Self {
            has_camera: profile.has_camera,
            has_motion_driver: profile.motor.is_some(),
            has_status_led: profile.status_led.is_some(),
            has_addressable_led: profile.addressable_led.is_some(),
            has_tca9555: false,
            tca9555_addr: None,
            has_lis2dh12: false,
            has_hdc2010: false,
            has_max17048: false,
        }
    }
}
