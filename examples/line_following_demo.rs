#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;
use picorobot::hal::{motors::{Motors, MotorId}, line_sensors::LineSensors};

// RTIC Monotonics
use rtic_monotonics::rp2040::prelude::*;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = rp_pico::hal::pac, dispatchers = [DMA_IRQ_0])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        motors: Motors,
        sensors: LineSensors,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("Line Following Demo Init");

        let pac = cx.device;
        let mut resets = pac.RESETS;
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        let _clocks = hal::clocks::init_clocks_and_plls(
            bsp::XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = hal::Sio::new(pac.SIO);
        let pins = bsp::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);

        // 1. Motors Configuration
        let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut resets);
        let mut pwm7 = pwm_slices.pwm7;
        pwm7.set_div_int(1);
        pwm7.set_top(65535);
        
        let _ = pins.gpio14.into_function::<hal::gpio::FunctionPwm>();
        let _ = pins.gpio15.into_function::<hal::gpio::FunctionPwm>();

        let motors = Motors::new(
            pwm7,
            pins.gpio11.into_push_pull_output(),
            pins.gpio10.into_push_pull_output(),
        );

        // 2. Line Sensors Configuration
        let sensors = LineSensors::new(
            pins.gpio22,
            pins.gpio21,
            pins.gpio20,
            pins.gpio19,
            pins.gpio18,
            pins.gpio26,
        );

        Mono::start(pac.TIMER, &mut resets);

        control_loop::spawn().ok();

        (Shared {}, Local { motors, sensors })
    }

    #[task(local = [motors, sensors])]
    async fn control_loop(cx: control_loop::Context) {
        let motors = cx.local.motors;
        let sensors = cx.local.sensors;
        let speed_scale = 0.5; // Scale factor to slow down the robot (1.0 = full speed)

        sensors.enable_emitters();
        Mono::delay(2.millis()).await;

        // --- Calibration Phase ---
        info!("Calibration: Move robot over line for 5s");
        for _ in 0..250 {
            sensors.calibrate();
            Mono::delay(20.millis()).await;
        }
        info!("Calibration Done. Starting line following...");

        loop {
            // Read scaled values (0-1000)
            let calibrated = sensors.read_calibrated();
            info!("Line: {}, {}, {}, {}, {}", calibrated[0], calibrated[1], calibrated[2], calibrated[3], calibrated[4]);
            
            // Basic logic with outer sensor support for sharper turns
            if calibrated[0] > 600 {
                // Far left sensor sees black: Sharp left turn
                motors.set_speed(MotorId::Left, -0.1 * speed_scale);
                motors.set_speed(MotorId::Right, 0.4 * speed_scale);
            } else if calibrated[4] > 600 {
                // Far right sensor sees black: Sharp right turn
                motors.set_speed(MotorId::Left, 0.4 * speed_scale);
                motors.set_speed(MotorId::Right, -0.1 * speed_scale);
            } else if calibrated[2] > 500 {
                // Center sensor sees black: Go forward
                motors.set_speed(MotorId::Left, 0.2 * speed_scale);
                motors.set_speed(MotorId::Right, 0.2 * speed_scale);
            } else if calibrated[1] > calibrated[3] {
                // Drift left
                motors.set_speed(MotorId::Left, 0.1 * speed_scale);
                motors.set_speed(MotorId::Right, 0.3 * speed_scale);
            } else {
                // Drift right
                motors.set_speed(MotorId::Left, 0.3 * speed_scale);
                motors.set_speed(MotorId::Right, 0.1 * speed_scale);
            }

            Mono::delay(10.millis()).await;
        }
    }
}
