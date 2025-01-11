//! This is the main entry point for a no_std, no_main Rust application targeting an STM32 microcontroller.
//!
//! The program configures the PA5 pin of GPIOA as an output pin in push-pull mode and toggles its state in an infinite loop with a simple delay.
//!
//! # Details
//!
//! - The program starts by defining the base address for GPIOA and the offsets for the mode, type, and state registers.
//! - It then sets PA5 as an output pin and configures it in push-pull mode.
//! - In the main loop, the program toggles the state of PA5 between high and low with a delay in between.
//!
//! # Safety
//!
//! - The program uses unsafe blocks to perform volatile memory operations, which are necessary for direct hardware manipulation.
//! - Care must be taken to ensure that the addresses and bit manipulations are correct to avoid undefined behavior.
//!
//! # Registers
//!
//! - `porta`: Base address for GPIOA registers.
//! - `gpioa_mode_reg_address`: Address for the GPIOA mode register.
//! - `gpioa_state_reg_address`: Address for the GPIOA state register.
//! - `gpioa_type_reg_address`: Address for the GPIOA type register.
//! - `rcc_ahb1enr`: Address for the RCC AHB1 peripheral clock enable register.
//!
//! # Constants
//!
//! - `gpioa5_output_mode`: Output mode for PA5 (0b01).
//! - `gpioa5_output_mode_bit_index`: Bit index for PA5 output mode (10).
//! - `gpioa5_pp_mode_bit_index`: Bit index for PA5 push-pull mode (5).
//! - `gpioa5_pp_mode`: Push-pull mode for PA5 (0).
//! - `gpioa5_high_state_bit_index`: Bit index for setting PA5 high (5).
//! - `gpioa5_low_state_bit_index`: Bit index for setting PA5 low (21).
//!
//! # Usage
//!
//! This code is intended to be run on an STM32 microcontroller. It requires the `cortex-m` and `cortex-m-rt` crates for the runtime and assembly instructions, and the `panic-halt` crate for handling panics.
#![no_std]
#![no_main]

use core::ptr::write_volatile;
use cortex_m::asm::nop;
use cortex_m_rt::entry;
use panic_halt as _;

#[entry]
fn main() -> ! {
    // PA configuration registers addresses
    let porta: *mut u32 = 0x40020000 as *mut u32;
    let gpioa_mode_reg_address: *mut u32 = (porta as u32 + 0x00) as *mut u32;
    let gpioa_state_reg_address: *mut u32 = (porta as u32 + 0x18) as *mut u32;
    let gpioa_type_reg_address: *mut u32 = (porta as u32 + 0x04) as *mut u32;

    // PA5 output mode
    let gpioa5_output_mode: u32 = 0b01;
    let gpioa5_output_mode_bit_index: u32 = 10;

    // PA5 push-pull mode
    let gpioa5_pp_mode_bit_index: u32 = 5;
    let gpioa5_pp_mode: u32 = 0;

    // PA5 state
    let gpioa5_high_state_bit_index: u32 = 5;
    let gpioa5_low_state_bit_index: u32 = 21;

    // Enable GPIOA clock (needed for PA configuration)
    let rcc_ahb1enr: *mut u32 = 0x40023830 as *mut u32;
    unsafe {
        let rcc_value = core::ptr::read_volatile(rcc_ahb1enr);
        write_volatile(rcc_ahb1enr, rcc_value | 1); // Habilitar GPIOA
    }

    // Configure PA5 as output, push-pull
    unsafe {
        // Set PA5 as output
        let mode_reg_value = core::ptr::read_volatile(gpioa_mode_reg_address);
        write_volatile(
            gpioa_mode_reg_address,
            (mode_reg_value & !(0b11 << gpioa5_output_mode_bit_index))
                | (gpioa5_output_mode << gpioa5_output_mode_bit_index),
        );

        // Set PA5 as push-pull
        let type_reg_value = core::ptr::read_volatile(gpioa_type_reg_address);
        write_volatile(
            gpioa_type_reg_address,
            type_reg_value | (gpioa5_pp_mode << gpioa5_pp_mode_bit_index),
        );
    }

    // Main loop
    let mut on_state: bool = false;
    loop {
        // Toggle LED status based on current state
        if on_state {
            unsafe {
                let state_reg_value = core::ptr::read_volatile(gpioa_state_reg_address);
                write_volatile(
                    gpioa_state_reg_address,
                    state_reg_value | (1 << gpioa5_high_state_bit_index),
                );
            }
        } else {
            unsafe {
                let state_reg_value = core::ptr::read_volatile(gpioa_state_reg_address);
                write_volatile(
                    gpioa_state_reg_address,
                    state_reg_value | (1 << gpioa5_low_state_bit_index),
                );
            }
        }

        // Toggle state
        on_state = !on_state;

        // Simple delay
        for _ in 0..1_000_000 {
            nop();
        }
    }
}
