use rp2040_hal as hal;
use hal::pio::{UninitStateMachine, StateMachine, PIO, PinDir, Rx, InstalledProgram, ValidStateMachine};
use hal::gpio::{Pin, FunctionPio0, PullUp, bank0::{Gpio8, Gpio9, Gpio12, Gpio13}};
use hal::pac::PIO0;
use defmt::info;

pub enum MotorId {
    Left,
    Right,
}

pub struct Encoders {
    _program_left: InstalledProgram<PIO0>,
    _program_right: InstalledProgram<PIO0>,
    _left_sm: StateMachine<(PIO0, hal::pio::SM0), hal::pio::Running>,
    _right_sm: StateMachine<(PIO0, hal::pio::SM1), hal::pio::Running>,
    left_rx: Rx<(PIO0, hal::pio::SM0)>,
    right_rx: Rx<(PIO0, hal::pio::SM1)>,
    left_count: i32,
    right_count: i32,
    last_left_state: u8,
    last_right_state: u8,
}

const QUAD_TABLE: [i8; 16] = [
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0,
];

impl Encoders {
    pub fn new(
        pio: &mut PIO<PIO0>,
        sm0: UninitStateMachine<(PIO0, hal::pio::SM0)>,
        sm1: UninitStateMachine<(PIO0, hal::pio::SM1)>,
        _left_pins: (Pin<Gpio12, FunctionPio0, PullUp>, Pin<Gpio13, FunctionPio0, PullUp>),
        _right_pins: (Pin<Gpio8, FunctionPio0, PullUp>, Pin<Gpio9, FunctionPio0, PullUp>),
    ) -> Self {
        // Reverting to hardcoded hex to remove pio-proc dependency while keeping the logic fixes.
        let program_data = [
            0xe03f, // 0: set x, 31       (Heartbeat)
            0xa0c1, // 1: mov isr, x
            0x8020, // 2: push noblock
            0xa043, // 3: mov y, null
            0xa0c3, // 4: mov isr, null   (Wrap target)
            0x4002, // 5: in pins, 2
            0xa026, // 6: mov x, isr
            0x00a9, // 7: jmp x != y, 9
            0x0004, // 8: jmp 4
            0xa041, // 9: mov y, x
            0x8020, // 10: push
        ];
        
        let program = pio::Program {
            code: {
                let mut instr = [0u16; 32];
                instr[..11].copy_from_slice(&program_data);
                instr.into()
            },
            origin: None, // Removing origin 0 to avoid relocation issues
            wrap: pio::Wrap { source: 10, target: 4 },
            side_set: pio::SideSet::new(false, 0, false),
        };

        let installed_left = pio.install(&program).unwrap();
        let installed_right = pio.install(&program).unwrap();
        
        // Left Encoder: GP12, GP13
        let (mut left_sm, left_rx, _) = hal::pio::PIOBuilder::from_installed_program(unsafe { installed_left.share() })
            .in_pin_base(12) 
            .in_shift_direction(hal::pio::ShiftDirection::Left)
            .clock_divisor_fixed_point(100, 0)
            .build(sm0);
            
        // Right Encoder: GP8, GP9
        let (mut right_sm, right_rx, _) = hal::pio::PIOBuilder::from_installed_program(unsafe { installed_right.share() })
            .in_pin_base(8)
            .in_shift_direction(hal::pio::ShiftDirection::Left)
            .clock_divisor_fixed_point(100, 0)
            .build(sm1);
            
        left_sm.set_pindirs([(12, PinDir::Input), (13, PinDir::Input)]);
        right_sm.set_pindirs([(8, PinDir::Input), (9, PinDir::Input)]);
        
        Self {
            _program_left: installed_left,
            _program_right: installed_right,
            _left_sm: left_sm.start(),
            _right_sm: right_sm.start(),
            left_rx,
            right_rx,
            left_count: 0,
            right_count: 0,
            last_left_state: 0,
            last_right_state: 0,
        }
    }

    fn update_single<T: ValidStateMachine>(
        rx: &mut Rx<T>,
        count: &mut i32,
        last_state: &mut u8,
        side: &str,
    ) {
        for _ in 0..16 {
            match rx.read() {
                Some(state) => {
                    if state == 31 {
                        info!("{} ENCODER HEARTBEAT", side);
                    } else {
                        let new_state = (state & 0x3) as u8;
                        let index = ((*last_state << 2) | new_state) as usize;
                        *count += QUAD_TABLE[index] as i32;
                        *last_state = new_state;
                    }
                }
                None => break,
            }
        }
    }

    pub fn update(&mut self) {
        Self::update_single(&mut self.right_rx, &mut self.right_count, &mut self.last_right_state, "Right");
    }

    pub fn get_counts(&self, motor: MotorId) -> i32 {
        match motor {
            MotorId::Left => self.left_count,
            MotorId::Right => self.right_count,
        }
    }
}