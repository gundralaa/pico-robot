use rp2040_hal as hal;
use hal::gpio::{Pin, FunctionSio, SioInput, SioOutput, PullNone, bank0::{Gpio18, Gpio19, Gpio20, Gpio21, Gpio22, Gpio26}};
use hal::pac;
use embedded_hal::digital::v2::OutputPin;

pub struct LineSensors {
    _pins: (
        Pin<Gpio22, FunctionSio<SioInput>, PullNone>, // Line 1
        Pin<Gpio21, FunctionSio<SioInput>, PullNone>, // Line 2
        Pin<Gpio20, FunctionSio<SioInput>, PullNone>, // Line 3
        Pin<Gpio19, FunctionSio<SioInput>, PullNone>, // Line 4
        Pin<Gpio18, FunctionSio<SioInput>, PullNone>, // Line 5
    ),
    emitter: Pin<Gpio26, FunctionSio<SioOutput>, PullNone>,
    min_values: [u32; 5],
    max_values: [u32; 5],
}

impl LineSensors {
    pub fn new<F22, P22, F21, P21, F20, P20, F19, P19, F18, P18, FE, PE>(
        p22: Pin<Gpio22, F22, P22>,
        p21: Pin<Gpio21, F21, P21>,
        p20: Pin<Gpio20, F20, P20>,
        p19: Pin<Gpio19, F19, P19>,
        p18: Pin<Gpio18, F18, P18>,
        emitter: Pin<Gpio26, FE, PE>,
    ) -> Self 
    where 
        F22: hal::gpio::Function, P22: hal::gpio::PullType,
        F21: hal::gpio::Function, P21: hal::gpio::PullType,
        F20: hal::gpio::Function, P20: hal::gpio::PullType,
        F19: hal::gpio::Function, P19: hal::gpio::PullType,
        F18: hal::gpio::Function, P18: hal::gpio::PullType,
        FE: hal::gpio::Function, PE: hal::gpio::PullType,
    {
        let mut emitter = emitter.into_push_pull_output();
        let _ = emitter.set_low();
        
        Self {
            _pins: (
                p22.into_floating_input(),
                p21.into_floating_input(),
                p20.into_floating_input(),
                p19.into_floating_input(),
                p18.into_floating_input(),
            ),
            emitter: emitter.into_pull_type(),
            min_values: [u32::MAX; 5],
            max_values: [0; 5],
        }
    }

    pub fn reset_calibration(&mut self) {
        self.min_values = [u32::MAX; 5];
        self.max_values = [0; 5];
    }

    pub fn calibrate(&mut self) {
        let raw = self.read();
        for i in 0..5 {
            if raw[i] < self.min_values[i] {
                self.min_values[i] = raw[i];
            }
            if raw[i] > self.max_values[i] {
                self.max_values[i] = raw[i];
            }
        }
    }

    pub fn enable_emitters(&mut self) {
        let _ = self.emitter.set_high();
    }

    pub fn disable_emitters(&mut self) {
        let _ = self.emitter.set_low();
    }

    /// Reads the reflectance sensors using the RC timing method.
    /// Returns raw timing counts for each of the 5 sensors.
    pub fn read(&mut self) -> [u32; 5] {
        let sio = unsafe { &*pac::SIO::ptr() };
        
        // Mask for GP18, 19, 20, 21, 22
        // Bits: 18, 19, 20, 21, 22
        let mask = (1 << 18) | (1 << 19) | (1 << 20) | (1 << 21) | (1 << 22);

        // 1. Drive pins High
        unsafe {
            sio.gpio_out_set().write(|w| w.bits(mask));
            sio.gpio_oe_set().write(|w| w.bits(mask));
        }

        // 2. Wait for capacitors to charge (~10us)
        cortex_m::asm::delay(1300); 

        // 3. Switch to Input (OE clear)
        unsafe {
            sio.gpio_oe_clr().write(|w| w.bits(mask));
        }

        let mut result = [2000u32; 5];
        let mut mask_remaining = mask;
        let max_count = 2000;

        // 4. Measure decay time
        for t in 0..max_count {
            let pins = sio.gpio_in().read().bits();
            let current_inputs = pins & mask;
            
            // Check each of the 5 pins (GP18-GP22)
            // Line 1: GP22, Line 2: GP21, Line 3: GP20, Line 4: GP19, Line 5: GP18
            for i in 0..5 {
                let pin_bit = 1 << (22 - i); // GP22 down to GP18
                if (mask_remaining & pin_bit) != 0 {
                    if (current_inputs & pin_bit) == 0 {
                        result[i] = t;
                        mask_remaining &= !pin_bit;
                    }
                }
            }
            
            if (mask_remaining & mask) == 0 {
                break;
            }
            cortex_m::asm::delay(125); 
        }

        result
    }

    /// Returns calibrated values from 0 to 1000.
    /// 0 represents maximum reflectance (white), 1000 represents minimum reflectance (black).
    pub fn read_calibrated(&mut self) -> [u32; 5] {
        let raw = self.read();
        let mut result = [0u32; 5];

        for i in 0..5 {
            let denominator = self.max_values[i] - self.min_values[i];
            if denominator == 0 {
                result[i] = 0;
            } else {
                let val = (raw[i].saturating_sub(self.min_values[i]) * 1000) / denominator;
                result[i] = val.min(1000);
            }
        }
        result
    }
}