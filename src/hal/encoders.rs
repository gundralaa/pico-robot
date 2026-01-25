use rp2040_hal as hal;
use hal::pio::{UninitStateMachine, StateMachine, PIO, PinDir, Rx, InstalledProgram, ValidStateMachine};
use hal::gpio::{Pin, FunctionPio0, PullUp, bank0::{Gpio8, Gpio9, Gpio12, Gpio13}};
use hal::pac::PIO0;
use defmt::info;
use pio_proc::pio_asm;

pub enum MotorId {
    Left,
    Right,
}

pub struct Encoders {
    _program: InstalledProgram<PIO0>,
    _left_sm: StateMachine<(PIO0, hal::pio::SM0), hal::pio::Running>,
    _right_sm: StateMachine<(PIO0, hal::pio::SM1), hal::pio::Running>,
    left_rx: Rx<(PIO0, hal::pio::SM0)>,
    right_rx: Rx<(PIO0, hal::pio::SM1)>,
}

impl Encoders {
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

    /// Read the latest count from the PIO. 
    /// This is non-blocking and returns the most recent value in the FIFO.
    fn read_latest<T: ValidStateMachine>(rx: &mut Rx<T>) -> i32 {
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
        (left, right)
    }
}
