# Rusty-STM32

Rusty-STM32 is a project for developing and running Rust applications on a STM32F401RE microcontroller. To complete this project, the YouTube tutorial series by [The Rusty Bits](https://www.youtube.com/@therustybits) was followed.

## Prerequisites

- Rust toolchain
- `cargo-generate` for creating new projects
- `probe-rs` for flashing and debugging

## Getting Started

1. **Install Rust toolchain:**

   ```sh
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   ```

2. **Install `cargo-generate`:**

   ```sh
   cargo install cargo-generate
   ```

3. **Install `probe-rs`:**

   ```sh
   cargo install probe-rs
   ```

4. **Install `llvm-tools` for `cargo-binutils`:**

   ```sh
   rustup component add llvm-tools
   ```

5. **Install `cargo-binutils`:**

   ```sh
    cargo install cargo-binutils
   ```

6. **Install `cargo-embed`:**

   ```sh
   cargo install cargo-embed
   ```

## Building the Project

To build the project, run the following command in the terminal:

```sh
cargo build
```

## Checking the Project

To check the project for errors, run:

```sh
cargo check
```

## Flashing the Firmware

To flash the firmware to the STM32 microcontroller, use:

```sh
cargo flash --release --chip STM32F401RE
```

You can optionally use the option `--connect-under-reset` to connect to the chip under reset state.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
