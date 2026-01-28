#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use rp_pico as bsp;

// RTIC Monotonics
use rtic_monotonics::rp2040::prelude::*;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = rp_pico::hal::pac, dispatchers = [DMA_IRQ_0])]
mod app {
    use super::*;
    use hal::gpio::FunctionUart;
    use hal::uart::{UartConfig, DataBits, StopBits, UartPeripheral};
    use hal::pac::UART0;
    use hal::clocks::Clock;
    use fugit::RateExtU32;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        uart: UartPeripheral<hal::uart::Enabled, UART0, (hal::gpio::Pin<hal::gpio::bank0::Gpio28, hal::gpio::FunctionUart, hal::gpio::PullDown>, hal::gpio::Pin<hal::gpio::bank0::Gpio29, hal::gpio::FunctionUart, hal::gpio::PullDown>)>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("UART Echo Demo Init");

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
        let pins = hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);

        // UART0 pins: GP28 (TX), GP29 (RX)
        let uart_pins = (
            pins.gpio28.into_function::<FunctionUart>(),
            pins.gpio29.into_function::<FunctionUart>(),
        );

        let mut uart = UartPeripheral::new(pac.UART0, uart_pins, &mut resets)
            .enable(
                UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        // Initialize monotonic
        Mono::start(pac.TIMER, &mut resets);

        // Spawn echo task
        echo_task::spawn().ok();

        (Shared {}, Local { uart })
    }

    #[task(local = [uart])]
    async fn echo_task(cx: echo_task::Context) {
        let uart = cx.local.uart;
        let mut rx_buf = [0u8; 1];
        let mut line_buf: heapless::Vec<u8, 64> = heapless::Vec::new();

        loop {
            // Read one byte (non-blocking style within the loop)
            if let Ok(_) = uart.read_full_blocking(&mut rx_buf) {
                let c = rx_buf[0];

                // If newline or carriage return, echo the buffer
                if c == b'\n' || c == b'\r' {
                    if !line_buf.is_empty() {
                        // Echo the line back
                        let _ = uart.write_full_blocking(&line_buf);
                        // Send a newline for clarity
                        let _ = uart.write_full_blocking(&[b'\r', b'\n']);
                        info!("Echoed line of length: {}", line_buf.len());
                        line_buf.clear();
                    }
                } else {
                    // Store character if there is space
                    if line_buf.push(c).is_err() {
                        // Buffer full: force an echo to clear space
                        let _ = uart.write_full_blocking(&line_buf);
                        let _ = uart.write_full_blocking(&[b'!', b'\r', b'\n']); // '!' to indicate overflow
                        line_buf.clear();
                        let _ = line_buf.push(c);
                    }
                }
            }
            Mono::delay(1.millis()).await;
        }
    }
}