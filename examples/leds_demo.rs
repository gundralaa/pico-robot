#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;
use picorobot::hal::leds::Leds;

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
        leds: Leds<(hal::gpio::Pin<hal::gpio::bank0::Gpio3, hal::gpio::FunctionSpi, hal::gpio::PullDown>, hal::gpio::Pin<hal::gpio::bank0::Gpio6, hal::gpio::FunctionSpi, hal::gpio::PullDown>)>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("LEDs Demo Init");

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
        // Pololu 3pi+ 2040 Pinout:
        // GP3: RGB LED Data (MOSI)
        // GP6: RGB LED Clock (SCLK)
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
        leds.set_brightness(5); // Low brightness for testing
        leds.set_all(0, 0, 0);
        leds.show();

        // Initialize monotonic
        Mono::start(pac.TIMER, &mut resets);

        led_task::spawn().ok();

        (Shared {}, Local { leds })
    }

    #[task(local = [leds])]
    async fn led_task(cx: led_task::Context) {
        let leds = cx.local.leds;
        let mut step = 0u8;

        loop {
            for i in 0..6 {
                let color = match (step + i as u8) % 3 {
                    0 => (255, 0, 0),   // Red
                    1 => (0, 255, 0),   // Green
                    _ => (0, 0, 255),   // Blue
                };
                leds.set_color(i, color.0, color.1, color.2);
            }
            leds.show();
            step = step.wrapping_add(1);
            Mono::delay(500.millis()).await;
        }
    }
}