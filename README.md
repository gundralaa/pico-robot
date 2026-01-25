# PicoRobot

PicoRobot is a Rust-based project for the Pololu 3pi+ 2040 Robot, utilizing the `rp-pico` HAL and the RTIC framework.

## Prerequisites

Before you begin, ensure you have the following installed:

1.  **Rust:** Install via [rustup.rs](https://rustup.rs/).
2.  **Thumbv6m Target:**
    ```bash
    rustup target add thumbv6m-none-eabi
    ```
3.  **probe-rs:** For flashing and debugging.
    ```bash
    cargo install probe-rs --features cli
    ```

## Configuration

Ensure that `.cargo/config.toml` is configured for your debug probe. The default configuration uses `probe-rs`.

```toml
[runner]
runner = "probe-rs run --chip RP2040 --protocol swd"
```

## Building and Running

### Main Application

To build and run the main application (`src/main.rs`):

```bash
cargo run --release
```

### Flashing Examples

To flash and run a specific example from the `examples/` directory, use the `--example` flag:

```bash
cargo run --release --example <example_name>
```

For example, to run the motors demo:

```bash
cargo run --release --example motors_demo
```

## Available Examples

| Example | Description |
| :--- | :--- |
| `motors_demo` | Basic motor control (forward, backward, stop) |
| `encoders_demo` | Reading wheel encoders via PIO (Legacy) |
| `encoders_hw_demo` | High-performance asynchronous encoder tracking |
| `leds_demo` | Controlling the RGB LEDs |
| `leds_off` | Utility to turn off all RGB LEDs |
| `display_demo` | Using the OLED display with `embedded-graphics` |
| `imu_demo` | Reading data from the onboard IMU |
| `target_velocity_demo` | Closed-loop velocity control example |
| `line_following_demo` | Line following implementation |

## Logging

This project uses `defmt` for logging. When running with `probe-rs`, the logs will be displayed in your terminal.
