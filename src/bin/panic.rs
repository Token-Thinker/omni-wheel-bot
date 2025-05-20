#![no_std]
#![no_main]

extern crate alloc;

// Minimal panic handler for no_std
use core::panic::PanicInfo;
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

use embassy_executor::{task, Spawner};
use smart_leds_trait::{SmartLedsWrite, RGB8};

// A dummy driver implementing SmartLedsWrite
struct Dummy;
impl SmartLedsWrite for Dummy {
    type Color = RGB8;
    type Error = ();
    fn write<I, O>(&mut self, _i: I) -> Result<(), ()>
    where
        I: IntoIterator<Item = RGB8>,
    {
        Ok(())
    }
}

#[task]
async fn led_task(mut driver: Dummy) {
    // This single call into the generic write + async machinery is enough
    let color = RGB8 { r: 1, g: 2, b: 3 };
    let _ = driver.write(core::iter::repeat(color).take(2));
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // Spawn the async task that trips the ICE
    spawner.spawn(led_task(Dummy)).unwrap();
    loop {
        // keep the runtime alive
    }
}