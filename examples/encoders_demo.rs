#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;
use picorobot::hal::encoders_old::{Encoders, MotorId};

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
        encoders: Encoders,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("Encoders Demo Init");

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

        // Init PIO
        let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut resets);
        
        let encoders = Encoders::new(
            &mut pio,
            sm0,
            sm1,
            (pins.gpio12.into_pull_up_input().into_function::<FunctionPio0>(), pins.gpio13.into_pull_up_input().into_function::<FunctionPio0>()),
            (pins.gpio8.into_pull_up_input().into_function::<FunctionPio0>(), pins.gpio9.into_pull_up_input().into_function::<FunctionPio0>()),
        );

        // Initialize monotonic
        Mono::start(pac.TIMER, &mut resets);

        // Spawn update task
        update::spawn().ok();

        (Shared {}, Local { encoders })
    }

    #[task(local = [encoders])]
    async fn update(cx: update::Context) {
        let encoders = cx.local.encoders;
        loop {
            encoders.update();
            let left = encoders.get_counts(MotorId::Left);
            let right = encoders.get_counts(MotorId::Right);
            info!("L: {}, R: {}", left, right);
            Mono::delay(100.millis()).await;
        }
    }
}