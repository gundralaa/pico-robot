#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;
use picorobot::hal::line_sensors::LineSensors;

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
        sensors: LineSensors,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("Line Sensors Demo Init");

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

        let sensors = LineSensors::new(
            pins.gpio16,
            pins.gpio17,
            pins.gpio18,
            pins.gpio19,
            pins.gpio20,
            pins.gpio26,
        );

        // Initialize monotonic
        Mono::start(pac.TIMER, &mut resets);

        // Spawn task
        sensor_read::spawn().ok();

        (Shared {}, Local { sensors })
    }

    #[task(local = [sensors])]
    async fn sensor_read(cx: sensor_read::Context) {
        let sensors = cx.local.sensors;
        
        // Enable emitters
        sensors.enable_emitters();
        Mono::delay(2.millis()).await; // Wait for emitters to stabilize

        loop {
            let values = sensors.read();
            info!("Sensors: {}, {}, {}, {}, {}", values[0], values[1], values[2], values[3], values[4]);
            
            Mono::delay(100.millis()).await;
        }
    }
}
