#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;
use picorobot::hal::{
    motors::Motors, 
    encoders::Encoders,
    closed_loop_motors::ClosedLoopMotors,
};

// RTIC Monotonics
use rtic_monotonics::rp2040::prelude::*;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = rp_pico::hal::pac, dispatchers = [DMA_IRQ_0])]
mod app {
    use super::*;
    use hal::pio::PIOExt;
    use hal::gpio::FunctionPio0;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        controller: ClosedLoopMotors,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("Target Velocity PI-Control Demo Init");

        let pac = cx.device;
        let mut resets = pac.RESETS;
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        let clocks = hal::clocks::init_clocks_and_plls(
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

        // 1. Initialize Motors
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

        // 2. Initialize Encoders (Hardware PIO)
        let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut resets);
        let encoders = Encoders::new(
            &mut pio,
            sm0,
            sm1,
            (pins.gpio12.into_pull_up_input().into_function::<FunctionPio0>(), pins.gpio13.into_pull_up_input().into_function::<FunctionPio0>()),
            (pins.gpio8.into_pull_up_input().into_function::<FunctionPio0>(), pins.gpio9.into_pull_up_input().into_function::<FunctionPio0>()),
        );

        // 3. Initialize Closed Loop Controller
        let mut controller = ClosedLoopMotors::new(motors, encoders);
        
        // Tune PI gains
        // Kp: 0.00025 (Proportional - immediate reaction)
        // Ki: 0.0005  (Integral - corrects steady-state error/drag)
        controller.set_gains(0.00025, 0.0005);

        // Initialize monotonic
        Mono::start(pac.TIMER, &mut resets);

        control_loop::spawn().ok();

        (Shared {}, Local { controller })
    }

    #[task(local = [controller])]
    async fn control_loop(cx: control_loop::Context) {
        let controller = cx.local.controller;
        let dt = 0.100; // 50Hz control loop
        let target_vel = 500.0; // counts per second
        let start_time = Mono::now();

        loop {
            controller.update_velocity(target_vel, target_vel, dt);
            
            let (l_count, r_count) = controller.get_raw_counts();
            let (l_speed, r_speed) = controller.get_current_speeds();
            let time_ms = (Mono::now() - start_time).to_millis();
            
            // CSV Output: Time(ms), Target, LeftSpeed, RightSpeed, LeftCount, RightCount
            info!("{},{},{},{},{},{}", time_ms, target_vel, l_speed, r_speed, l_count, r_count);
            
            Mono::delay(100.millis()).await;
        }
    }
}
