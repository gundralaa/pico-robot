# Pololu 3pi+ 2040 Robot - HAL Architecture

## 1. System Overview

This project aims to provide a Hardware Abstraction Layer (HAL) for the Pololu 3pi+ 2040 Robot using the Rust programming language. The system is built upon the `rp2040-hal` and `rtic` frameworks.

## 2. Hardware Mapping

Based on the verified specs, the pinout is defined as follows:

| Component | Function | GPIO Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Motors** | Motor 1 (Right) PWM | GP14 | PWM Slice 7A |
| | Motor 1 (Right) Dir | GP10 | Digital Output |
| | Motor 2 (Left) PWM | GP15 | PWM Slice 7B |
| | Motor 2 (Left) Dir | GP11 | Digital Output |
| **Encoders** | Right A | GP8 | |
| | Right B | GP9 | |
| | Left A | GP12 | |
| | Left B | GP13 | |
| **Line Sensors** | Line 1 (DN1) | GP22 | RC Timing |
| | Line 2 (DN2) | GP21 | RC Timing |
| | Line 3 (DN3) | GP20 | RC Timing |
| | Line 4 (DN4) | GP19 | RC Timing |
| | Line 5 (DN5) | GP18 | RC Timing |
| | Emitter Control | GP26 | Digital Output |
| **RGB LEDs** | Data | GP3 | SPI MOSI (SPI0 TX) |
| | Clock | GP6 | SPI SCK (SPI0 SCK) |
| | Type | APA102 | Compatible (DotStar) |
| **User Interface**| Button A | GP14 | (Shared with Motor? Check) |
| | Button B | GP15 | (Shared with Motor? Check) |
| | Button C | GP0 | (Shared with Display DC?) |
| | Buzzer | GP21 | PWM (GP7 in reference?) |
| **Display** | SH1106 / SSD1306 | | 128x64 OLED |
| | Data/Command | GP0 | |
| | Reset | GP1 | |
| | SPI SCK | GP2 | |
| | SPI MOSI | GP3 | |
| **IMU** | LSM6DSO (Accel/Gyro) | I2C0 | Addr: 0x6B |
| | LIS3MDL (Mag) | I2C0 | Addr: 0x1E |

## 3. Module Design

The project will be organized into the following modules within `src/hal/`:

### 3.1. `motors.rs`
- **Responsibility:** Control speed and direction of the two DC motors.
- **Implementation:**
    - Use `rp2040_hal::pwm` to generate PWM signals.
    - Uses PWM Slice 7 for both motors.
    - `set_speed(motor: MotorId, speed: f32)`: speed range -1.0 to 1.0.
    - Handle direction pins based on the sign of the speed.
    - `stop()`: Disable PWM / Set to 0.

### 3.2. `encoders.rs`
- **Responsibility:** Track wheel rotation.
- **Implementation:**
    - **Asynchronous PIO Quadrature Decoder**: High-performance implementation that offloads counting to the PIO hardware.
    - **Logic**: Uses a 16-instruction jump table in PIO memory to process every phase transition (A/B) and maintain a 32-bit signed counter in the PIO's `Y` register.
    - **Reliability**: This method eliminates race conditions and prevents dropped pulses that can occur with CPU-based polling, especially at high speeds.
    - `get_counts() -> (i32, i32)`: Returns the current net counts for both motors.

### 3.3. `closed_loop_motors.rs`
- **Responsibility:** Maintain constant wheel velocity using encoder feedback.
- **Implementation:**
    - **PID Controller**: Implements a Proportional-Integral-Derivative (PID) controller for each motor.
    - **Integration**: Ingests both `Motors` and `Encoders` modules.
    - **Velocity Control**: Calculates current velocity from encoder deltas and adjusts PWM output to match a target setpoint (counts/sec).
    - `update_velocity(left_target: f32, right_target: f32, dt: f32)`: Main entry point for the control loop.

### 3.3. `line_sensors.rs`
- **Responsibility:** Read the 5 reflectance sensors.
- **Implementation:**
    - RC Timing method:
        1. Drive pins High.
        2. Wait ~10us.
        3. Switch pins to Input (High-Z).
        4. Measure time for voltage to decay to Low.
    - `read() -> [u32; 5]`: Returns raw decay times.

### 3.4. `leds.rs`
- **Responsibility:** Control the 6 APA102-compatible RGB LEDs.
- **Implementation:**
    - SPI. GP3 (Data) and GP6 (Clock).
    - `set_color(index: usize, r: u8, g: u8, b: u8)`.

### 3.5. `imu.rs`
- **Responsibility:** Interface with the onboard IMU sensors.

### 3.6. `display.rs`
- **Responsibility:** Text/Graphics output.
- **Implementation:**
    - `embedded-graphics` support.
    - Driver: `sh1106` over SPI.

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
```