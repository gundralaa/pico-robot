use rp2040_hal as hal;
use hal::spi::{Spi, Enabled, ValidSpiPinout};
use hal::pac::SPI0;
use embedded_hal::blocking::spi::Write;

// 3pi+ 2040 has 6 APA102/SK9822 LEDs
const NUM_LEDS: usize = 6;

pub struct Leds<P: ValidSpiPinout<SPI0>> {
    spi: Spi<Enabled, SPI0, P>,
    colors: [(u8, u8, u8); NUM_LEDS],
    brightness: u8,
}

impl<P: ValidSpiPinout<SPI0>> Leds<P> {
    pub fn new(spi: Spi<Enabled, SPI0, P>) -> Self {
        Self {
            spi,
            colors: [(0, 0, 0); NUM_LEDS],
            brightness: 31, // Max brightness (0-31)
        }
    }

    pub fn set_color(&mut self, index: usize, r: u8, g: u8, b: u8) {
        if index < NUM_LEDS {
            self.colors[index] = (r, g, b);
        }
    }

    pub fn set_all(&mut self, r: u8, g: u8, b: u8) {
        for i in 0..NUM_LEDS {
            self.colors[i] = (r, g, b);
        }
    }

    pub fn set_brightness(&mut self, brightness: u8) {
        self.brightness = brightness.min(31);
    }

    pub fn show(&mut self) {
        let start_frame = [0u8; 4];
        let _ = self.spi.write(&start_frame);

        let global_byte = 0xE0 | self.brightness;
        for (r, g, b) in self.colors.iter() {
            let frame = [global_byte, *b, *g, *r];
            let _ = self.spi.write(&frame);
        }

        let end_frame = [0xFFu8; 4];
        let _ = self.spi.write(&end_frame);
    }
}