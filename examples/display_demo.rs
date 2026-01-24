#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;
use picorobot::hal::display::Display;

// RTIC Monotonics
use rtic_monotonics::rp2040::prelude::*;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = rp_pico::hal::pac, dispatchers = [DMA_IRQ_0])]
mod app {
    use super::*;
    use hal::gpio::FunctionSpi;
    use hal::spi::Spi;
    use hal::clocks::Clock;
    use hal::fugit::RateExtU32;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        display: Display,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("Display Demo Init");

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

        // Init SPI0 for Display
        let mosi_pin = pins.gpio3.into_function::<FunctionSpi>();
        let sclk_pin = pins.gpio2.into_function::<FunctionSpi>();
        
        let spi = Spi::new(pac.SPI0, (mosi_pin, sclk_pin));
        let spi = spi.init(
            &mut resets,
            clocks.peripheral_clock.freq(),
            8u32.MHz(),
            embedded_hal::spi::MODE_0,
        );

        let dc = pins.gpio0.into_push_pull_output();
        let reset = pins.gpio1.into_push_pull_output();
        let cs = pins.gpio28.into_push_pull_output();

        let mut delay = cortex_m::delay::Delay::new(cx.core.SYST, clocks.system_clock.freq().to_Hz());

        let mut display = Display::new(
            spi,
            dc,
            cs,
            reset,
            &mut delay,
        );

        display.clear();
        display.write_text("PicoRobot", 10, 20);
        display.write_text("Display Demo", 10, 40);
        display.flush();

        // Initialize monotonic
        Mono::start(pac.TIMER, &mut resets);

        (Shared {}, Local { display })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}