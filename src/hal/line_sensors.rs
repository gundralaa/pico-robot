use rp2040_hal as hal;
use hal::gpio::{Pin, FunctionSio, SioInput, SioOutput, PullNone, bank0::{Gpio16, Gpio17, Gpio18, Gpio19, Gpio20, Gpio26}};
use hal::pac;
use embedded_hal::digital::v2::OutputPin;

pub struct LineSensors {
    _pins: (
        Pin<Gpio16, FunctionSio<SioInput>, PullNone>,
        Pin<Gpio17, FunctionSio<SioInput>, PullNone>,
        Pin<Gpio18, FunctionSio<SioInput>, PullNone>,
        Pin<Gpio19, FunctionSio<SioInput>, PullNone>,
        Pin<Gpio20, FunctionSio<SioInput>, PullNone>,
    ),
    emitter: Pin<Gpio26, FunctionSio<SioOutput>, PullNone>,
}

impl LineSensors {
    pub fn new<F16, P16, F17, P17, F18, P18, F19, P19, F20, P20, FE, PE>(
        p16: Pin<Gpio16, F16, P16>,
        p17: Pin<Gpio17, F17, P17>,
        p18: Pin<Gpio18, F18, P18>,
        p19: Pin<Gpio19, F19, P19>,
        p20: Pin<Gpio20, F20, P20>,
        emitter: Pin<Gpio26, FE, PE>,
    ) -> Self 
    where 
        F16: hal::gpio::Function, P16: hal::gpio::PullType,
        F17: hal::gpio::Function, P17: hal::gpio::PullType,
        F18: hal::gpio::Function, P18: hal::gpio::PullType,
        F19: hal::gpio::Function, P19: hal::gpio::PullType,
        F20: hal::gpio::Function, P20: hal::gpio::PullType,
        FE: hal::gpio::Function, PE: hal::gpio::PullType,
    {
        let mut emitter = emitter.into_push_pull_output();
        let _ = emitter.set_low();
        
        Self {
            _pins: (
                p16.into_floating_input(),
                p17.into_floating_input(),
                p18.into_floating_input(),
                p19.into_floating_input(),
                p20.into_floating_input(),
            ),
            emitter: emitter.into_pull_type(),
        }
    }

    pub fn enable_emitters(&mut self) {
        let _ = self.emitter.set_high();
    }

    pub fn disable_emitters(&mut self) {
        let _ = self.emitter.set_low();
    }

    pub fn read(&mut self) -> [u32; 5] {
        let sio = unsafe { &*pac::SIO::ptr() };
        let mask = 0x1F << 16;

        unsafe {
            sio.gpio_out_set().write(|w| w.bits(mask));
            sio.gpio_oe_set().write(|w| w.bits(mask));
        }

        cortex_m::asm::delay(1300); 

        unsafe {
            sio.gpio_oe_clr().write(|w| w.bits(mask));
        }

        let mut result = [2000u32; 5];
        let mut mask_remaining = 0x1Fu32;
        let max_count = 2000;

        for t in 0..max_count {
            let pins = sio.gpio_in().read().bits();
            let inputs = (pins >> 16) & 0x1F;
            
            for i in 0..5 {
                let bit = 1 << i;
                if (mask_remaining & bit) != 0 {
                    if (inputs & bit) == 0 {
                        result[i] = t;
                        mask_remaining &= !bit;
                    }
                }
            }
            
            if mask_remaining == 0 {
                break;
            }
            cortex_m::asm::delay(125); 
        }

        result
    }
}