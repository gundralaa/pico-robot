# Pololu 3pi+ 2040 Robot - HAL Architecture

## 1. System Overview

This project aims to provide a Hardware Abstraction Layer (HAL) for the Pololu 3pi+ 2040 Robot using the Rust programming language. The system is built upon the `rp2040-hal` and `rtic` frameworks.

## 2. Hardware Mapping

Based on the verified specs, the pinout is defined as follows:

| Component | Function | GPIO Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Motors** | Motor 1 (Left) PWM | GP8 | PWM Slice 4A |
| | Motor 1 (Left) Dir | GP9 | Digital Output |
| | Motor 2 (Right) PWM | GP6 | PWM Slice 3A |
| | Motor 2 (Right) Dir | GP7 | Digital Output |
| **Encoders** | Left A | GP10 | |
| | Left B | GP11 | |
| | Right A | GP12 | |
| | Right B | GP13 | |
| **Line Sensors** | Sensor 1 (DN1) | GP16 | RC Timing (PIO candidate) |
| | Sensor 2 (DN2) | GP17 | RC Timing |
| | Sensor 3 (DN3) | GP18 | RC Timing |
| | Sensor 4 (DN4) | GP19 | RC Timing |
| | Sensor 5 (DN5) | GP20 | RC Timing |
| | Emitter Control | GP26 | Digital Output |
| **RGB LEDs** | Data | GP3 | SPI MOSI (SPI0 TX) |
| | Clock | GP2 | SPI SCK (SPI0 SCK) |
| | Type | APA102 | Compatible (DotStar) |
| **User Interface**| Button A | GP14 | Active Low |
| | Button B | GP15 | Active Low |
| | Button C | GP23 | Active Low |
| | Buzzer | GP21 | PWM (check pin) |
| **Display** | SH1106 / SSD1306 | | 128x64 OLED |
| | Data/Command | GP0 | |
| | Reset | GP1 | |
| | I2C SDA | GP4 | I2C0 SDA |
| | I2C SCL | GP5 | I2C0 SCL |
| **IMU** | LSM6DSO (Accel/Gyro) | I2C0 | Addr: 0x6B |
| | LIS3MDL (Mag) | I2C0 | Addr: 0x1E |

## 3. Module Design

The project will be organized into the following modules within `src/hal/`:

### 3.1. `motors.rs`
- **Responsibility:** Control speed and direction of the two DC motors.
- **Implementation:**
    - Use `rp2040_hal::pwm` to generate PWM signals.
    - `set_speed(motor: MotorId, speed: f32)`: speed range -1.0 to 1.0.
    - Handle direction pins based on the sign of the speed.
    - `stop()`: Disable PWM / Set to 0.

### 3.2. `encoders.rs`
- **Responsibility:** Track wheel rotation.
- **Implementation:**
    - PIO (Programmable I/O) quadrature decoder (high performance, low CPU overhead). **Preferred**.
    - `get_counts(motor: MotorId) -> i32`: Returns net counts.

### 3.3. `line_sensors.rs`
- **Responsibility:** Read the 5 reflectance sensors.
- **Implementation:**
    - RC Timing method:
        1. Drive pins High.
        2. Wait ~10us.
        3. Switch pins to Input (High-Z).
        4. Measure time for voltage to decay to Low.
    - This is a perfect candidate for **PIO** to read all 5 sensors in parallel without blocking the CPU.
    - `calibrate()`: for setting the max and min values
    - `get_reflectance(sensor: SensorId) -> f32` from 0.0 to 1.0

### 3.4. `leds.rs`
- **Responsibility:** Control the 6 APA102-compatible RGB LEDs.
- **Implementation:**
    - SPI. Since GP2/GP3 are SPI0 SCK/TX, `rp2040_hal::spi` is the best choice.
    - `set_color(index: usize, r: u8, g: u8, b: u8)`.
    - Note that the display and the leds share a spi bus and switching needs to be enabled

### 3.5. `imu.rs`
- **Responsibility:** Interface with the onboard IMU sensors.
- **Implementation:**
    - Use `shared-bus` or RTIC resources to share the I2C0 bus.
    - Drivers: `lsm6dso` and `lis3mdl` crates (if available) or custom implementation.

### 3.6. `display.rs`
- **Responsibility:** Text/Graphics output.
- **Implementation:**
    - `embedded-graphics` support.
    - Driver: `sh1106` or `ssd1306` crate over I2C, plus GPIOs for Reset and D/C.

## 4. Directory Structure

```
src/
├── main.rs         # Application logic (RTIC)
└── hal/
    ├── mod.rs      # Module exports
    ├── motors.rs
    ├── encoders.rs
    ├── line_sensors.rs
    ├── leds.rs
    ├── imu.rs
    └── display.rs
examples/
    ├── motors_demo.rs
    ├── encoders_demo.rs
    ├── line_sensors_demo.rs
    ├── leds_demo.rs
    ├── display_demo.rs
    ├── imu_demo.rs
    ├── target_velocity_demo.rs
    └── line_following_demo.rs
```

## 5. Development Plan & Examples

We will implement the modules incrementally, verifying each with a dedicated example in the `examples/` directory.

### 5.1. `motors_demo.rs`
- **Goal:** Verify motor control.
- **Logic:**
    - Initialize PWMs and Direction pins.
    - Ramp up speed M1 Forward, then Stop.
    - Ramp up speed M1 Backward, then Stop.
    - Repeat for M2.

### 5.2. `encoders_demo.rs`
- **Goal:** Verify PIO quadrature decoding.
- **Logic:**
    - Initialize PIO programs for both encoders.
    - Periodically (e.g., every 100ms) read counts and print to `defmt` log.

### 5.3. `line_sensors_demo.rs`
- **Goal:** Verify sensor reading via PIO.
- **Logic:**
    - Initialize PIO for sensor array.
    - Print raw sensor values to log.

### 5.4. `leds_demo.rs`
- **Goal:** Verify RGB LEDs.
- **Logic:**
    - Cycle through Red, Green, Blue on all 6 LEDs.

### 5.5. `display_demo.rs`
- **Goal:** Verify OLED display.
- **Logic:**
    - Initialize I2C and Display driver.
    - Draw text "PicoRobot" and a simple shape.

### 5.6. `imu_demo.rs`
- **Goal:** Verify IMU communication.
- **Logic:**
    - Read Accel/Gyro ID and values.

### 5.7. `target_velocity_demo.rs` (Integrated Application)
- **Goal:** Implement closed-loop velocity control.
- **Logic:**
    - Use encoder feedback and a PID controller to maintain a target velocity.

### 5.8. `line_following_demo.rs` (Integrated Application)
- **Goal:** Implement line following behavior.
- **Logic:**
    - Use line sensor data and a PID controller to follow a line.