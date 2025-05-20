/// Abstract motor‐/sensor‐driver interface.
pub trait WheelDriver
{
    type Error;

    /// Read the current wheel speeds (rad/s).
    fn read_wheel_speeds(&mut self) -> Result<[f32; 3], Self::Error>;

    /// Command new wheel duties or speeds (rad/s or normalized duty).
    fn set_wheel_speeds(
        &mut self,
        speeds: [f32; 3],
    ) -> Result<(), Self::Error>;
}
