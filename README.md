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

## Running Examples

To flash and run a specific example from the `examples/` directory, use the `--example` flag:

```bash
cargo run --release --example <example_name>
```

### Available Examples

| Example | Description |
| :--- | :--- |
| `motors_demo` | Basic motor control (forward, backward, stop) |
| `encoders_demo` | Reading wheel encoders via PIO (Software Decoding) |
| `encoders_hw_demo` | High-performance asynchronous encoder tracking (Hardware Decoding) |
| `target_velocity_demo` | PID-controlled constant velocity using encoder feedback |
| `line_sensors_demo` | Calibrating and reading the 5-channel reflectance sensor array |
| `line_following_demo` | Full line-following application with calibration and PID steering |
| `leds_demo` | Controlling the RGB LEDs |
| `leds_off` | Utility to turn off all RGB LEDs |
| `display_demo` | Using the OLED display with `embedded-graphics` |
| `imu_demo` | Reading data from the onboard IMU |
| `uart_echo_demo` | Echoes characters received on UART0 (GP28/GP29) |
| `blink_led` | Basic blinky example (migrated from main) |

## Testing UART Echo

To test the `uart_echo_demo`, you will need a USB-to-TTL serial adapter connected to the robot's UART pins (**GP28/TX** and **GP29/RX**).

1.  **Connect the Adapter:**
    -   Adapter RX -> Robot GP28 (TX)
    -   Adapter TX -> Robot GP29 (RX)
    -   Adapter GND -> Robot GND
2.  **Flash the Demo:**
    ```bash
    cargo run --release --example uart_echo_demo
    ```
3.  **Connect using `screen`:**
    Identify your serial device (e.g., `/dev/tty.usbserial-XYZ`) and run:
    ```bash
    screen /dev/tty.<your_serial_device> 115200
    ```
    *To exit screen, press `Ctrl+A` then `K` then `Y`.*

## Hardware Modules

The project provides high-level drivers for the robot's subsystems in `src/hal/`.

| Module | Description | Key Features |
| :--- | :--- | :--- |
| `motors` | Dual DC Motor Control | PWM speed control, Direction handling |
| `encoders` | Quadrature Encoders | PIO-based hardware counting, Zero CPU overhead |
| `closed_loop_motors` | Velocity Control | PID controller integration for constant speed |
| `line_sensors` | Reflectance Array | 5-channel RC timing, Calibration, Scaled output |
| `display` | OLED Screen | SH1106 driver, `embedded-graphics` integration |
| `leds` | RGB LEDs | APA102 (DotStar) driver via SPI |
| `imu` | Inertial Sensors | Accelerometer and Gyroscope access |

## Logging

This project uses `defmt` for efficient logging. When running with `probe-rs`, logs are automatically displayed in your terminal.

### Log Levels
You can set the log level at runtime by setting the `DEFMT_LOG` environment variable:

```bash
DEFMT_LOG=trace cargo run --release --example target_velocity_demo
```

Available levels: `error`, `warn`, `info`, `debug`, `trace`.

### Macros
Use these macros in your code to print logs:
-   `error!("Error message")`
-   `warn!("Warning message")`
-   `info!("Info message")`
-   `debug!("Debug message")`
-   `trace!("Trace message")`