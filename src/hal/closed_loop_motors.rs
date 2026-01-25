use super::motors::{Motors, MotorId};
use super::encoders::Encoders;

pub struct PiController {
    pub kp: f32,
    pub ki: f32,
    pub integral: f32,
    pub out_min: f32,
    pub out_max: f32,
}

impl PiController {
    pub fn new(kp: f32, ki: f32, out_min: f32, out_max: f32) -> Self {
        Self {
            kp,
            ki,
            integral: 0.0,
            out_min,
            out_max,
        }
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;
        
        // Accumulate Integral term
        self.integral += error * dt;
        
        // Anti-windup: Clamp the integral term independently to prevent it from growing indefinitely
        // when the actuator is saturated. This is a simple but effective method.
        // Often usually clamped to the output limits, but can be tighter.
        self.integral = self.integral.clamp(self.out_min, self.out_max);

        let output = (self.kp * error) + (self.ki * self.integral);
        
        output.clamp(self.out_min, self.out_max)
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
    }
}

pub struct ClosedLoopMotors {
    motors: Motors,
    encoders: Encoders,
    left_controller: PiController,
    right_controller: PiController,
    last_left_count: i32,
    last_right_count: i32,
    current_left_speed: f32,
    current_right_speed: f32,
}

impl ClosedLoopMotors {
    pub fn new(motors: Motors, encoders: Encoders) -> Self {
        Self {
            motors,
            encoders,
            // Initial gains: Kp=0.001, Ki=0.0 (disabled by default until tuned)
            left_controller: PiController::new(0.001, 0.0, -1.0, 1.0),
            right_controller: PiController::new(0.001, 0.0, -1.0, 1.0),
            last_left_count: 0,
            last_right_count: 0,
            current_left_speed: 0.0,
            current_right_speed: 0.0,
        }
    }

    /// Update the velocity control loop.
    /// target_velocity is in encoder counts per second.
    /// dt is the time elapsed since the last update in seconds.
    pub fn update_velocity(&mut self, left_target: f32, right_target: f32, dt: f32) {
        let (curr_left, curr_right) = self.encoders.get_counts();
        
        // Calculate current velocity (counts/sec)
        let left_vel = (curr_left - self.last_left_count) as f32 / dt;
        let right_vel = (curr_right - self.last_right_count) as f32 / dt;
        
        self.last_left_count = curr_left;
        self.last_right_count = curr_right;
        
        // Update PI controllers
        let left_out = self.left_controller.update(left_target, left_vel, dt);
        let right_out = self.right_controller.update(right_target, right_vel, dt);
        
        self.current_left_speed = left_out;
        self.current_right_speed = right_out;

        // Set motor speeds
        self.motors.set_speed(MotorId::Left, left_out);
        self.motors.set_speed(MotorId::Right, right_out);
    }

    pub fn get_current_speeds(&self) -> (f32, f32) {
        (self.current_left_speed, self.current_right_speed)
    }

    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.left_controller.kp = kp;
        self.left_controller.ki = ki;
        self.right_controller.kp = kp;
        self.right_controller.ki = ki;
    }

    pub fn stop(&mut self) {
        self.current_left_speed = 0.0;
        self.current_right_speed = 0.0;
        self.left_controller.reset();
        self.right_controller.reset();
        self.motors.stop();
    }
    
    pub fn get_raw_counts(&mut self) -> (i32, i32) {
        self.encoders.get_counts()
    }
}