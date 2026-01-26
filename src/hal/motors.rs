use rp2040_hal as hal;
use hal::pwm::{Pwm7, Slice, FreeRunning};
#[cfg(not(test))]
use hal::gpio::{Pin, FunctionSio, SioOutput, PullDown, bank0::{Gpio10, Gpio11}};
use embedded_hal::PwmPin;
use embedded_hal::digital::v2::OutputPin;

#[cfg(test)]
use self::mocks::{MockPwmHandle, MockPinHandle};

pub enum MotorId {
    Left,
    Right,
}

pub trait PwmSlice {
    fn set_ph_correct(&mut self);
    fn enable(&mut self);
    fn set_duty_a(&mut self, duty: u16);
    fn set_duty_b(&mut self, duty: u16);
}

impl PwmSlice for Slice<Pwm7, FreeRunning> {
    fn set_ph_correct(&mut self) {
        self.set_ph_correct();
    }
    fn enable(&mut self) {
        self.enable();
    }
    fn set_duty_a(&mut self, duty: u16) {
        self.channel_a.set_duty(duty);
    }
    fn set_duty_b(&mut self, duty: u16) {
        self.channel_b.set_duty(duty);
    }
}

pub struct Motors {
    #[cfg(not(test))]
    pwm: Slice<Pwm7, FreeRunning>,
    #[cfg(test)]
    pub pwm: MockPwmHandle,

    #[cfg(not(test))]
    left_dir: Pin<Gpio11, FunctionSio<SioOutput>, PullDown>,
    #[cfg(test)]
    pub left_dir: MockPinHandle,

    #[cfg(not(test))]
    right_dir: Pin<Gpio10, FunctionSio<SioOutput>, PullDown>,
    #[cfg(test)]
    pub right_dir: MockPinHandle,
}

impl Motors {
    #[cfg(not(test))]
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

    #[cfg(test)]
    pub fn new_test() -> Self {
        let mut pwm = MockPwmHandle::new();
        // Initialize as in real new()
        pwm.set_ph_correct();
        pwm.enable();

        let mut m = Self {
            pwm,
            left_dir: MockPinHandle::new(),
            right_dir: MockPinHandle::new(),
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
                self.pwm.set_duty_b(duty);
            }
            MotorId::Right => {
                // Right Motor is on Channel A
                if speed >= 0.0 {
                    let _ = self.right_dir.set_high();
                } else {
                    let _ = self.right_dir.set_low();
                }
                self.pwm.set_duty_a(duty);
            }
        }
    }

    pub fn stop(&mut self) {
        self.pwm.set_duty_a(0);
        self.pwm.set_duty_b(0);
    }
}

#[cfg(test)]
pub mod mocks {
    use super::*;
    use std::cell::RefCell;
    use std::rc::Rc;

    pub struct MockPwm {
        pub duty_a: u16,
        pub duty_b: u16,
        pub ph_correct: bool,
        pub enabled: bool,
    }

    impl MockPwm {
        pub fn new() -> Self {
            Self {
                duty_a: 0,
                duty_b: 0,
                ph_correct: false,
                enabled: false,
            }
        }
    }

    #[derive(Clone)]
    pub struct MockPwmHandle(pub Rc<RefCell<MockPwm>>);

    impl MockPwmHandle {
        pub fn new() -> Self {
            Self(Rc::new(RefCell::new(MockPwm::new())))
        }
    }

    impl PwmSlice for MockPwmHandle {
        fn set_ph_correct(&mut self) {
            self.0.borrow_mut().ph_correct = true;
        }
        fn enable(&mut self) {
            self.0.borrow_mut().enabled = true;
        }
        fn set_duty_a(&mut self, duty: u16) {
            self.0.borrow_mut().duty_a = duty;
        }
        fn set_duty_b(&mut self, duty: u16) {
            self.0.borrow_mut().duty_b = duty;
        }
    }

    pub struct MockPin {
        pub high: bool,
    }

    impl MockPin {
        pub fn new() -> Self {
            Self { high: false }
        }
    }

    #[derive(Clone)]
    pub struct MockPinHandle(pub Rc<RefCell<MockPin>>);

    impl MockPinHandle {
        pub fn new() -> Self {
            Self(Rc::new(RefCell::new(MockPin::new())))
        }
    }

    impl OutputPin for MockPinHandle {
        type Error = ();
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.0.borrow_mut().high = false;
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.0.borrow_mut().high = true;
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motors_initialization() {
        let motors = Motors::new_test();

        assert!(motors.pwm.0.borrow().ph_correct);
        assert!(motors.pwm.0.borrow().enabled);
        assert_eq!(motors.pwm.0.borrow().duty_a, 0); // stop() called in new
        assert_eq!(motors.pwm.0.borrow().duty_b, 0);
    }

    #[test]
    fn test_set_speed_left() {
        let mut motors = Motors::new_test();

        // Forward
        motors.set_speed(MotorId::Left, 0.5);
        assert!(motors.left_dir.0.borrow().high);
        assert_eq!(motors.pwm.0.borrow().duty_b, 32767); // 0.5 * 65535

        // Backward
        motors.set_speed(MotorId::Left, -0.5);
        assert!(!motors.left_dir.0.borrow().high);
        assert_eq!(motors.pwm.0.borrow().duty_b, 32767);
    }

    #[test]
    fn test_set_speed_right() {
        let mut motors = Motors::new_test();

        // Forward
        motors.set_speed(MotorId::Right, 0.5);
        assert!(motors.right_dir.0.borrow().high);
        assert_eq!(motors.pwm.0.borrow().duty_a, 32767); // 0.5 * 65535

        // Backward
        motors.set_speed(MotorId::Right, -0.5);
        assert!(!motors.right_dir.0.borrow().high);
        assert_eq!(motors.pwm.0.borrow().duty_a, 32767);
    }
}
