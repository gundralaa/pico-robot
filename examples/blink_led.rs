#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use embedded_hal::digital::v2::ToggleableOutputPin;
use hal::gpio::bank0::Gpio25;
use hal::gpio::{FunctionSio, Pin, PullDown, SioOutput};
use rp_pico as bsp;

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
        led: Pin<Gpio25, FunctionSio<SioOutput>, PullDown>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("RTIC 2.0 Init");

        let pac = cx.device; // removed mut
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

        let led = pins.led.into_push_pull_output();

        // Initialize monotonic
        Mono::start(pac.TIMER, &mut resets);

        // Spawn blink task
        blink::spawn().ok();

        (Shared {}, Local { led })
    }

    #[task(local = [led])]
    async fn blink(cx: blink::Context) {
        let led = cx.local.led;
        loop {
            info!("Toggle!");
            led.toggle().unwrap();
            Mono::delay(500.millis()).await;
        }
    }
}
