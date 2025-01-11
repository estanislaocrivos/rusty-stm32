#![no_std]
#![no_main]

use cortex_m::asm::nop;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{self as hal, gpio::GpioExt};
#[entry]
fn main() -> ! {
    // Get access to the device peripherals
    let p = hal::pac::Peripherals::take().unwrap();
    // Get access to the GPIOA peripheral
    let porta = p.GPIOA.split();
    // Configure PA5 as a push-pull output
    let mut pina5 = porta.pa5.into_push_pull_output();
    loop {
        pina5.set_high();
        for _ in 0..500_000 {
            nop();
        }
        pina5.set_low();
        for _ in 0..500_000 {
            nop();
        }
    }
}
