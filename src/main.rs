#![no_std]
#![no_main]

use core::ptr::write_volatile;
// use cortex_m::asm::nop;
use cortex_m_rt::entry;
use panic_halt as _;
#[entry]
fn main() -> ! {
    let porta: *mut u32 = 0x40020000 as *mut u32;
    let gpioa_mode_reg_address: *mut u32 = (porta as u32 + 0x00) as *mut u32;
    let gpioa_state_reg_address: *mut u32 = (porta as u32 + 0x18) as *mut u32;
    let gpioa_type_reg_address: *mut u32 = (porta as u32 + 0x04) as *mut u32;
    let gpioa_pupd_reg_address: *mut u32 = (porta as u32 + 0x0C) as *mut u32;
    let gpioa5_output_mode: u32 = 0x00000400;
    let gpioa5_pp_mode: u32 = 0x00000000;
    let gpioa5_pd_mode: u32 = 0x00000000;
    let gpioa5_high: u32 = 0x00000020;
    // let gpioa5_low: u32 = 0x00200000;

    let rcc_ahb1enr: *mut u32 = 0x40023830 as *mut u32;
    unsafe {
        let rcc_value = core::ptr::read_volatile(rcc_ahb1enr);
        write_volatile(rcc_ahb1enr, rcc_value | (1 << 0)); // Habilitar GPIOA
    }

    unsafe {
        // Set PA5 as output
        write_volatile(gpioa_mode_reg_address, gpioa5_output_mode);
        // Set PA5 as push-pull
        write_volatile(gpioa_type_reg_address, gpioa5_pp_mode);
        // Set PA5 as pull-down
        write_volatile(gpioa_pupd_reg_address, gpioa5_pd_mode);
        // Set PA5 high
        write_volatile(gpioa_state_reg_address, gpioa5_high);
    }
    // let mut is_on: bool = false;
    // loop {
    //     // Set PA5 high
    //     unsafe {
    //         if is_on {
    //             write_volatile(gpioa_state_reg_address, gpioa5_high);
    //         } else {
    //             write_volatile(gpioa_state_reg_address, gpioa5_low);
    //         }
    //         is_on = !is_on;
    //     }
    //     for _ in 0..400_000 {
    //         nop();
    //     }
    // }
    loop {}
}
