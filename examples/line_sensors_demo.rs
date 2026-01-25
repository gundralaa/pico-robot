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
        info!("Line Sensors Calibration Demo Init");

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
            pins.gpio22,
            pins.gpio21,
            pins.gpio20,
            pins.gpio19,
            pins.gpio18,
            pins.gpio26,
        );

        // Initialize monotonic
        Mono::start(pac.TIMER, &mut resets);

        // Spawn task
        sensor_task::spawn().ok();

        (Shared {}, Local { sensors })
    }

    #[task(local = [sensors])]
    async fn sensor_task(cx: sensor_task::Context) {
        let sensors = cx.local.sensors;
        
        // Enable emitters
        sensors.enable_emitters();
        Mono::delay(2.millis()).await;

        // --- Calibration Phase ---
        info!("Starting Calibration (10 seconds)...");
        info!("Move the robot over white and black surfaces.");
        
        // Loop for approximately 500 samples (500 * 20ms = 10s)
        for _ in 0..500 {
            sensors.calibrate();
            Mono::delay(20.millis()).await;
        }
        
        info!("Calibration Complete!");

        // --- Main Loop ---
        loop {
            let calibrated = sensors.read_calibrated();
            info!("Calibrated: 1:{}, 2:{}, 3:{}, 4:{}, 5:{}", 
                calibrated[0], calibrated[1], calibrated[2], calibrated[3], calibrated[4]);
            
            Mono::delay(100.millis()).await;
        }
    }
}