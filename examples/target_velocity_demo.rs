#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;
use picorobot::hal::{motors::Motors, encoders::Encoders};

// RTIC Monotonics
use rtic_monotonics::rp2040::prelude::*;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = rp_pico::hal::pac, dispatchers = [DMA_IRQ_0])]
mod app {
    use super::*;
    use hal::pio::PIOExt;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        motors: Motors,
        encoders: Encoders,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("Target Velocity Demo Init");

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

        // Motors
        let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut resets);
        let mut pwm4 = pwm_slices.pwm4;
        pwm4.set_div_int(1); // Fast PWM
        pwm4.set_top(20000); // 20000 cycles
        let mut pwm3 = pwm_slices.pwm3;
        pwm3.set_div_int(1);
        pwm3.set_top(20000);
        let _ = pins.gpio8.into_function::<hal::gpio::FunctionPwm>();
        let _ = pins.gpio6.into_function::<hal::gpio::FunctionPwm>();

        let motors = Motors::new(
            pwm4,
            pwm3,
            pins.gpio9.into_push_pull_output(),
            pins.gpio7.into_push_pull_output(),
        );

        // Encoders
        let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut resets);
        let encoders = Encoders::new(
            &mut pio,
            sm0,
            sm1,
            (pins.gpio10.into_pull_up_input().into_function(), pins.gpio11.into_pull_up_input().into_function()),
            (pins.gpio12.into_pull_up_input().into_function(), pins.gpio13.into_pull_up_input().into_function()),
        );

        // Initialize monotonic
        Mono::start(pac.TIMER, &mut resets);

        control_loop::spawn().ok();

        (Shared {}, Local { motors, encoders })
    }

    #[task(local = [motors, encoders])]
    async fn control_loop(cx: control_loop::Context) {
        loop {
            // PID logic would go here
            cx.local.encoders.update();
            Mono::delay(10.millis()).await;
        }
    }
}