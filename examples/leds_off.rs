#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;
use picorobot::hal::leds::Leds;

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
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("Turning LEDs Off...");

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

        // Init SPI0 for LEDs
        let mosi = pins.gpio3.into_function::<FunctionSpi>();
        let sclk = pins.gpio6.into_function::<FunctionSpi>();
        
        let spi = Spi::new(pac.SPI0, (mosi, sclk));
        let spi = spi.init(
            &mut resets,
            clocks.peripheral_clock.freq(),
            2u32.MHz(),
            embedded_hal::spi::MODE_0,
        );

        let mut leds = Leds::new(spi);
        leds.set_all(0, 0, 0); // Black/Off
        leds.show();
        
        info!("LEDs are now off.");

        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}