use rp2040_hal as hal;
use hal::pwm::{Pwm7, Slice, FreeRunning};
use hal::gpio::{Pin, FunctionSio, SioOutput, PullDown, bank0::{Gpio10, Gpio11}};
use embedded_hal::PwmPin;
use embedded_hal::digital::v2::OutputPin;

pub enum MotorId {
    Left,
    Right,
}

pub struct Motors {
    pwm: Slice<Pwm7, FreeRunning>,
    left_dir: Pin<Gpio11, FunctionSio<SioOutput>, PullDown>,
    right_dir: Pin<Gpio10, FunctionSio<SioOutput>, PullDown>,
}

impl Motors {
    pub fn new(
        mut pwm: Slice<Pwm7, FreeRunning>,
        left_dir: Pin<Gpio11, FunctionSio<SioOutput>, PullDown>,
        right_dir: Pin<Gpio10, FunctionSio<SioOutput>, PullDown>,
    ) -> Self {
        // Initialize PWM channels
        pwm.set_ph_correct();
        pwm.enable();
        
        let mut m = Self {
            pwm,
            left_dir,
            right_dir,
        };
        
        m.stop();
        m
    }

    pub fn set_speed(&mut self, motor: MotorId, speed: f32) {
        let speed = speed.clamp(-1.0, 1.0);
        let abs_speed = if speed < 0.0 { -speed } else { speed };
        let duty = (abs_speed * 65535.0) as u16;

        match motor {
            MotorId::Left => {
                // Left Motor is on Channel B
                if speed >= 0.0 {
                    let _ = self.left_dir.set_high();
                } else {
                    let _ = self.left_dir.set_low();
                }
                self.pwm.channel_b.set_duty(duty);
            }
            MotorId::Right => {
                // Right Motor is on Channel A
                if speed >= 0.0 {
                    let _ = self.right_dir.set_high();
                } else {
                    let _ = self.right_dir.set_low();
                }
                self.pwm.channel_a.set_duty(duty);
            }
        }
    }

    pub fn stop(&mut self) {
        self.pwm.channel_a.set_duty(0);
        self.pwm.channel_b.set_duty(0);
    }
}
