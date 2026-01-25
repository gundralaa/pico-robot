#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use rp_pico as bsp;

#[rtic::app(device = rp_pico::hal::pac, dispatchers = [DMA_IRQ_0])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        info!("PicoRobot Application Started");
        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}