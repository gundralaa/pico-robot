#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;
use picorobot::hal::motors::{Motors, MotorId};
use embedded_hal::digital::v2::OutputPin;

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
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("Motors Demo Init");

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

        // Init PWMs
        let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut resets);

        // Configure Motors
        // PWM Slice 7 controls both motors
        // Channel A (GP14) -> Right Motor PWM
        // Channel B (GP15) -> Left Motor PWM
        let mut pwm7 = pwm_slices.pwm7;
        pwm7.set_div_int(20); // Adjust as needed
        pwm7.set_top(65535);

        // Configure PWM pins
        let _ = pins.gpio14.into_function::<hal::gpio::FunctionPwm>();
        let _ = pins.gpio15.into_function::<hal::gpio::FunctionPwm>();

        // Fix: Prevent LEDs from flashing due to floating pins picking up motor noise
        // GP6 is RGB LED Clock, GP3 is RGB LED Data
        let mut led_clock = pins.gpio6.into_push_pull_output();
        let mut led_data = pins.gpio3.into_push_pull_output();
        let _ = led_clock.set_low();
        let _ = led_data.set_low();

        // Configure Direction pins
        // GP10 -> Right Motor Dir
        // GP11 -> Left Motor Dir
        let right_dir = pins.gpio10.into_push_pull_output();
        let left_dir = pins.gpio11.into_push_pull_output();

        let motors = Motors::new(
            pwm7,
            left_dir,
            right_dir,
        );

        // Initialize monotonic
        Mono::start(pac.TIMER, &mut resets);

        // Spawn motor test task
        motor_test::spawn().ok();

        (Shared {}, Local { motors })
    }

    #[task(local = [motors])]
    async fn motor_test(cx: motor_test::Context) {
        let motors = cx.local.motors;
        
        loop {
            info!("Left Motor Forward");
            motors.set_speed(MotorId::Left, 0.5);
            Mono::delay(2000.millis()).await;
            
            info!("Left Motor Backward");
            motors.set_speed(MotorId::Left, -0.5);
            Mono::delay(2000.millis()).await;
            
            motors.stop();
            Mono::delay(1000.millis()).await;

            info!("Right Motor Forward");
            motors.set_speed(MotorId::Right, 0.5);
            Mono::delay(2000.millis()).await;
            
            info!("Right Motor Backward");
            motors.set_speed(MotorId::Right, -0.5);
            Mono::delay(2000.millis()).await;
            
            motors.stop();
            Mono::delay(1000.millis()).await;
        }
    }
}