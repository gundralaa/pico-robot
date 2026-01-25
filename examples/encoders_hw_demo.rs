#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;
use picorobot::hal::encoders::Encoders;
use picorobot::hal::display::Display;

// RTIC Monotonics
use rtic_monotonics::rp2040::prelude::*;

use heapless::String;
use core::fmt::Write;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = rp_pico::hal::pac, dispatchers = [DMA_IRQ_0])]
mod app {
    use super::*;
    use hal::pio::PIOExt;
    use hal::gpio::FunctionPio0;
    use hal::gpio::FunctionSpi;
    use hal::spi::Spi;
    use hal::clocks::Clock;
    use hal::fugit::RateExtU32;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        encoders: Encoders,
        display: Display,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("Encoders HW Demo Init");

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

        // 1. Initialize Display
        let mosi_pin = pins.gpio3.into_function::<FunctionSpi>();
        let sclk_pin = pins.gpio2.into_function::<FunctionSpi>();
        let spi = Spi::new(pac.SPI0, (mosi_pin, sclk_pin)).init(
            &mut resets,
            clocks.peripheral_clock.freq(),
            8u32.MHz(),
            embedded_hal::spi::MODE_0,
        );

        let mut display = Display::new(
            spi,
            pins.gpio0.into_push_pull_output(),
            pins.gpio28.into_push_pull_output(),
            pins.gpio1.into_push_pull_output(),
            &mut cortex_m::delay::Delay::new(cx.core.SYST, clocks.system_clock.freq().to_Hz()),
        );
        display.clear();
        display.write_text("Encoders HW", 10, 10);
        display.flush();

        // 2. Initialize Encoders (PIO)
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

        (Shared {}, Local { encoders, display })
    }

    #[task(local = [encoders, display])]
    async fn update(cx: update::Context) {
        let encoders = cx.local.encoders;
        let display = cx.local.display;
        let mut buf_l: String<32> = String::new();
        let mut buf_r: String<32> = String::new();

        loop {
            let (left, right) = encoders.get_counts();
            
            buf_l.clear();
            buf_r.clear();
            write!(buf_l, "Left:  {}", left).unwrap();
            write!(buf_r, "Right: {}", right).unwrap();

            display.clear();
            display.write_text("Encoders HW", 10, 10);
            display.write_text(&buf_l, 10, 30);
            display.write_text(&buf_r, 10, 50);
            display.flush();

            Mono::delay(50.millis()).await;
        }
    }
}