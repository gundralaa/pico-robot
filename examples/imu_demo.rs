#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;
use picorobot::hal::imu::Imu;

// RTIC Monotonics
use rtic_monotonics::rp2040::prelude::*;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = rp_pico::hal::pac, dispatchers = [DMA_IRQ_0])]
mod app {
    use super::*;
    use hal::gpio::FunctionI2c;
    use hal::clocks::Clock;
    use fugit::RateExtU32;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        imu: Imu<(hal::gpio::Pin<hal::gpio::bank0::Gpio4, hal::gpio::FunctionI2c, hal::gpio::PullUp>, hal::gpio::Pin<hal::gpio::bank0::Gpio5, hal::gpio::FunctionI2c, hal::gpio::PullUp>)>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("IMU Demo Init");

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

        // Init I2C0 for IMU
        let sda = pins.gpio4.into_pull_up_input().into_function::<FunctionI2c>();
        let scl = pins.gpio5.into_pull_up_input().into_function::<FunctionI2c>();
        
        let i2c = hal::I2C::i2c0(
            pac.I2C0,
            sda,
            scl,
            400.kHz(),
            &mut resets,
            &clocks.system_clock,
        );

        let mut imu = Imu::new(i2c);
        if imu.init().is_ok() {
            info!("IMU initialized successfully");
        } else {
            info!("IMU initialization failed!");
        }

        // Initialize monotonic
        Mono::start(pac.TIMER, &mut resets);

        (Shared {}, Local { imu })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}