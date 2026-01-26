use rp2040_hal as hal;
#[cfg(not(test))]
use hal::pio::{UninitStateMachine, StateMachine, PIO, PinDir, InstalledProgram};
use hal::pio::{Rx, ValidStateMachine};
#[cfg(not(test))]
use hal::gpio::{Pin, FunctionPio0, PullUp, bank0::{Gpio8, Gpio9, Gpio12, Gpio13}};
#[cfg(not(test))]
use hal::pac::PIO0;
#[cfg(not(test))]
use defmt::info;
#[cfg(not(test))]
use pio_proc::pio_asm;

#[cfg(test)]
use self::mocks::MockRxHandle;

pub enum MotorId {
    Left,
    Right,
}

pub trait EncoderRx {
    fn read(&mut self) -> Option<u32>;
}

impl<T: ValidStateMachine> EncoderRx for Rx<T> {
    fn read(&mut self) -> Option<u32> {
        self.read()
    }
}

pub struct Encoders {
    #[cfg(not(test))]
    _program: InstalledProgram<PIO0>,
    #[cfg(not(test))]
    _left_sm: StateMachine<(PIO0, hal::pio::SM0), hal::pio::Running>,
    #[cfg(not(test))]
    _right_sm: StateMachine<(PIO0, hal::pio::SM1), hal::pio::Running>,

    #[cfg(not(test))]
    left_rx: Rx<(PIO0, hal::pio::SM0)>,
    #[cfg(test)]
    pub left_rx: MockRxHandle,

    #[cfg(not(test))]
    right_rx: Rx<(PIO0, hal::pio::SM1)>,
    #[cfg(test)]
    pub right_rx: MockRxHandle,
}

impl Encoders {
    #[cfg(not(test))]
    pub fn new(
        pio: &mut PIO<PIO0>,
        sm0: UninitStateMachine<(PIO0, hal::pio::SM0)>,
        sm1: UninitStateMachine<(PIO0, hal::pio::SM1)>,
        _left_pins: (Pin<Gpio12, FunctionPio0, PullUp>, Pin<Gpio13, FunctionPio0, PullUp>),
        _right_pins: (Pin<Gpio8, FunctionPio0, PullUp>, Pin<Gpio9, FunctionPio0, PullUp>),
    ) -> Self {
        // High-performance Hardware Quadrature Decoder
        // This program maintains the encoder count in the Y register and pushes it to the RX FIFO.
        // It uses a 16-entry jump table starting at origin 0 to handle all state transitions.
        let pio_program = pio_asm!(
            "
            .origin 0
            ; Jump Table (Indices 0-15 correspond to [prev_A, prev_B, curr_A, curr_B])
            ; 00 state (Previous)
                jmp update    ; read 00 (No change)
                jmp decrement ; read 01
                jmp increment ; read 10
                jmp update    ; read 11 (Invalid/Skip)
            ; 01 state (Previous)
                jmp increment ; read 00
                jmp update    ; read 01 (No change)
                jmp update    ; read 10 (Invalid/Skip)
                jmp decrement ; read 11
            ; 10 state (Previous)
                jmp decrement ; read 00
                jmp update    ; read 01 (Invalid/Skip)
                jmp update    ; read 10 (No change)
                jmp increment ; read 11
            ; 11 state (Previous)
                jmp update    ; read 00 (Invalid/Skip)
                jmp increment ; read 01
                jmp decrement ; read 10
                jmp update    ; read 11 (No change)

            decrement:
                jmp y--, update

            .wrap_target
            update:
                mov isr, y
                push noblock

            sample_pins:
                out isr, 2    ; Move previous state from OSR to ISR
                in pins, 2    ; Read current state from pins into ISR
                mov osr, isr  ; Save [prev, curr] for next iteration
                mov pc, isr   ; Jump to the table entry (0-15)

            increment:
                mov y, ~y     ; 2's complement increment: Y = ~(~Y - 1)
                jmp y--, increment_cont
            increment_cont:
                mov y, ~y
            .wrap
            "
        );

        // Install at origin 0 as required by the absolute jump table
        let installed = pio.install(&pio_program.program).unwrap();
        
        // Left Encoder: GP12, GP13
        let (mut left_sm, left_rx, _) = hal::pio::PIOBuilder::from_installed_program(unsafe { installed.share() })
            .in_pin_base(12) 
            .in_shift_direction(hal::pio::ShiftDirection::Left)
            .clock_divisor_fixed_point(10, 0) // Run fast to capture all transitions
            .build(sm0);
            
        // Right Encoder: GP8, GP9
        let (mut right_sm, right_rx, _) = hal::pio::PIOBuilder::from_installed_program(unsafe { installed.share() })
            .in_pin_base(8)
            .in_shift_direction(hal::pio::ShiftDirection::Left)
            .clock_divisor_fixed_point(10, 0)
            .build(sm1);
            
        left_sm.set_pindirs([(12, PinDir::Input), (13, PinDir::Input)]);
        right_sm.set_pindirs([(8, PinDir::Input), (9, PinDir::Input)]);
        
        info!("Encoders (HW): Starting PIO state machines at offset {}", installed.offset());
        
        Self {
            _program: installed,
            _left_sm: left_sm.start(),
            _right_sm: right_sm.start(),
            left_rx,
            right_rx,
        }
    }

    #[cfg(test)]
    pub fn new_test() -> Self {
        Self {
            left_rx: MockRxHandle::new(),
            right_rx: MockRxHandle::new(),
        }
    }

    /// Read the latest count from the PIO.
    /// This is non-blocking and returns the most recent value in the FIFO.
    fn read_latest(rx: &mut impl EncoderRx) -> i32 {
        let mut count = None;
        // TODO: use a match method here in order to avoid blocking
        while let Some(val) = rx.read() {
            count = Some(val as i32);
        }
        count.unwrap_or(0) // Note: In a real app, you'd store and return the last known value
    }

    pub fn get_counts(&mut self) -> (i32, i32) {
        let left = Self::read_latest(&mut self.left_rx);
        let right = Self::read_latest(&mut self.right_rx);
        (-left, -right)
    }
}

#[cfg(test)]
pub mod mocks {
    use super::*;
    use std::cell::RefCell;
    use std::rc::Rc;
    use std::collections::VecDeque;

    #[derive(Clone)]
    pub struct MockRxHandle(pub Rc<RefCell<VecDeque<u32>>>);

    impl MockRxHandle {
        pub fn new() -> Self {
            Self(Rc::new(RefCell::new(VecDeque::new())))
        }

        pub fn push(&self, value: u32) {
            self.0.borrow_mut().push_back(value);
        }
    }

    impl EncoderRx for MockRxHandle {
        fn read(&mut self) -> Option<u32> {
            self.0.borrow_mut().pop_front()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_counts() {
        let mut encoders = Encoders::new_test();
        let left_rx = encoders.left_rx.clone();
        let right_rx = encoders.right_rx.clone();

        // Nothing to read
        let (left, right) = encoders.get_counts();
        assert_eq!(left, 0);
        assert_eq!(right, 0);

        // Push some data
        left_rx.push(100);
        left_rx.push(101); // Last one is 101

        right_rx.push(50);
        right_rx.push(51); // Last one is 51

        let (left, right) = encoders.get_counts();
        // get_counts returns negative values
        assert_eq!(left, -101);
        assert_eq!(right, -51);

        // FIFO drained
        let (left, right) = encoders.get_counts();
        assert_eq!(left, 0);
        assert_eq!(right, 0);
    }
}
