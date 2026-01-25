# Gemini Project: PicoRobot

This file provides a comprehensive overview of the PicoRobot project, its structure, and how to interact with it.

## Project Overview

PicoRobot is an embedded software project designed to run on a Raspberry Pi Pico microcontroller. The project is written in Rust and utilizes the `rp-pico` and `rp2040-hal` crates for hardware abstraction. It employs the Real-Time Interrupt-driven Concurrency (RTIC) framework to manage tasks, as evidenced by the `cortex-m-rtic` dependency and the `#[rtic::app]` macro in `src/main.rs`.

The current functionality of the project is a simple "hello world" for embedded systems: it blinks the onboard LED of the Raspberry Pi Pico. The application is configured for a release profile with size optimization (`opt-level = "s"`), which is a common practice for embedded systems with limited resources.

## Building and Running

### Building

To build the project, use the standard Cargo build command:

```bash
cargo build --release
```

This will create an ELF file in `target/thumbv6m-none-eabi/release/picorobot`.

### Running on Device

To run the code on a Raspberry Pi Pico, you will need a debug probe that is supported by `probe-rs`. The `.cargo/config.toml` file provides a template for the runner command.

1.  **Install `probe-rs`:**
    ```bash
    cargo install probe-rs --features cli
    ```

2.  **Uncomment and configure the runner:**
    In `.cargo/config.toml`, uncomment the `runner` line and ensure it's configured for your specific debug probe. For example, if you are using the Raspberry Pi Pico's on-board debugger, the command might look like this:

    ```toml
    [runner]
    runner = "probe-rs run --chip RP2040 --protocol swd"
    ```

3.  **Run the code:**
    With the runner configured, you can flash and run the application with:

    ```bash
    cargo run --release
    ```

## Development Conventions

### Logging

The project uses the `defmt` crate for logging. `defmt` provides a flexible and efficient logging framework for embedded devices. You can view the `defmt` output by using `probe-rs`.

### Code Structure

-   `src/main.rs`: The main application entry point and RTIC application definition.
-   `Cargo.toml`: Defines project dependencies, features, and metadata.
-   `.cargo/config.toml`: Contains build and runner configurations.
-   `memory.x`: A linker script that defines the memory layout for the target device.

### Guidelines

Search the web for existing solutions and libraries before implementing new drivers for hardware.
