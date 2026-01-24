use rp2040_hal as hal;
use hal::gpio::{Pin, FunctionSpi, FunctionSio, SioOutput, PullDown, bank0::{Gpio0, Gpio1, Gpio2, Gpio3, Gpio28}};
use hal::Spi;
use hal::spi::Enabled;
use embedded_hal::digital::v2::OutputPin;
use hal::pac::SPI0;
use embedded_hal::blocking::delay::DelayMs;
use sh1106::{prelude::*, Builder};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

pub type DisplaySpi = Spi<Enabled, SPI0, (Pin<Gpio3, FunctionSpi, PullDown>, Pin<Gpio2, FunctionSpi, PullDown>)>;
pub type DisplayDC = Pin<Gpio0, FunctionSio<SioOutput>, PullDown>;
pub type DisplayCS = Pin<Gpio28, FunctionSio<SioOutput>, PullDown>;
pub type DisplayReset = Pin<Gpio1, FunctionSio<SioOutput>, PullDown>;

pub struct Display {
    driver: GraphicsMode<SpiInterface<DisplaySpi, DisplayDC, DisplayCS>>,
}

impl Display {
    pub fn new<D: DelayMs<u8>>(
        spi: DisplaySpi,
        dc: DisplayDC,
        cs: DisplayCS,
        mut reset: DisplayReset,
        delay: &mut D,
    ) -> Self {
        // Reset sequence
        let _ = reset.set_low();
        delay.delay_ms(10);
        let _ = reset.set_high();
        delay.delay_ms(10);

        let mut display: GraphicsMode<_> = Builder::new()
            .with_size(DisplaySize::Display128x64)
            .connect_spi(spi, dc, cs)
            .into();

        display.init().unwrap();
        display.flush().unwrap();

        Self { driver: display }
    }

    pub fn clear(&mut self) {
        self.driver.clear();
    }

    pub fn flush(&mut self) {
        self.driver.flush().unwrap();
    }

    pub fn write_text(&mut self, text: &str, x: i32, y: i32) {
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        Text::new(text, Point::new(x, y), style)
            .draw(&mut self.driver)
            .unwrap();
    }
}