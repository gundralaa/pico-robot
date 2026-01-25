# Differential Drive Control System Design

## Problem Statement
When commanding identical velocities to the left and right motors (`target_velocity`), the robot drifts or curves because of physical differences between the two drivetrains (e.g., motor efficiency, friction, wheel diameter variance). As a result, the encoder counts diverge over time (`Left_Count > Right_Count` or vice versa).

## Solution 1: Cascaded Control Architecture (Sync Controller)

To solve this, we need a control system with two layers:
1.  **Inner Loop (Velocity Control):** Maintains the target speed for each wheel (P-Controller already implemented).
2.  **Outer Loop (Synchronization/Heading Control):** Adjusts the *relative* target speeds to ensure the robot drives straight or follows a specific curve.

### 1. The Sync Controller (Straight Line)

For driving straight, the goal is for the *distance traveled* (total encoder counts) by both wheels to be equal (or maintain a fixed offset).

**Error Term:**
```text
Sync_Error = Left_Total_Counts - Right_Total_Counts
```

**Correction:**
We calculate a correction term based on this error and apply it differentially to the velocity targets.

```text
Correction = K_sync * Sync_Error
```

**Applying Correction:**
*   **Left Target** = `Base_Velocity - Correction`
*   **Right Target** = `Base_Velocity + Correction`

*Logic:* If Left is ahead (`Sync_Error > 0`), the correction is positive. Left target decreases (slows down), Right target increases (speeds up).

### 2. Implementation Strategy

We will extend the `ClosedLoopMotors` module to include this logic.

#### Revised `ClosedLoopMotors` Structure

```rust
pub struct ClosedLoopMotors {
    // ... existing fields ...
    sync_controller: PController, // New controller for synchronization
    sync_enabled: bool,
}
```

#### New Method: `update_differential_drive`

Instead of setting raw left/right speeds, the user sets a `linear_velocity` and `angular_velocity` (or just `linear` for straight driving).

```rust
pub fn update_drive(&mut self, linear_velocity: f32, angular_velocity: f32, dt: f32) {
    let (curr_left, curr_right) = self.encoders.get_counts();
    
    // 1. Calculate Sync Correction (only if driving straight, i.e., angular ~= 0)
    let mut correction = 0.0;
    if angular_velocity.abs() < 0.001 && self.sync_enabled {
        let error = (curr_left - self.start_left) - (curr_right - self.start_right);
        correction = self.sync_controller.update(0.0, error as f32);
    }

    // 2. Apply Correction to Base Targets
    // For a differential drive: 
    // V_left = V_linear - (Width * V_angular / 2)
    // V_right = V_linear + (Width * V_angular / 2)
    // Here we simplify and just add the correction term directly to velocity targets.
    
    let left_target = linear_velocity - correction;
    let right_target = linear_velocity + correction;

    // 3. Feed into existing Velocity PID
    self.update_velocity(left_target, right_target, dt);
}
```

## Solution 2: Alternative Approach - Integral (I) Term

An alternative to the explicit Sync Controller is to enhance the existing velocity controller by adding an **Integral (I)** term to the P-Controller, creating a **PI Controller**.

### Concept
A P-controller requires a non-zero error to generate output. This means if one motor has more friction, it will naturally settle at a slightly lower speed than the target to maintain the necessary torque (steady-state error). The I-term accumulates this error over time (`Integral += Error * dt`) and adds it to the output.

If the left wheel is consistently slower than the target, the integral builds up, increasing the power to that motor until the speed error is eliminated.

### Comparative Analysis

| Feature | Solution 1: Sync Controller (Cascaded) | Solution 2: Velocity PI Controller |
| :--- | :--- | :--- |
| **Correction Target** | Corrects **Relative Position** (Distance). | Corrects **Absolute Velocity** (Speed). |
| **Drift Handling** | **Excellent.** By syncing total counts, it inherently corrects for any past drift. If the robot turns slightly, it will speed up one wheel to "catch up" the distance. | **Good.** It ensures both wheels spin at exactly X RPM. However, it *does not* guarantee they have traveled the same distance. Small transient errors accumulate into heading drift over time. |
| **Tuning** | **Complex.** Requires tuning the inner velocity loop first, then the outer sync loop. | **Medium.** Requires tuning Kp and Ki. Finding a Ki that removes error without causing "windup" or oscillation can be tricky. |
| **Robustness** | Robust against uneven terrain or friction changes because it cares about *net displacement*. | Sensitive to transient load changes. The I-term takes time to react and settle. |
| **Implementation** | Requires new logic/module structure (`update_differential_drive`). | Requires modifying existing `PController` to `PIController` (adding state `integral`). |

### Recommendation

For **straight-line driving tasks**, the **Sync Controller (Solution 1)** is superior because it explicitly minimizes the heading error (difference in distance traveled).

For **general velocity control** where straightness isn't the only priority (e.g., maintaining speed on a slope), the **PI Controller (Solution 2)** is standard.

A hybrid approach is often best: Use PI controllers for the inner velocity loops to ensure good speed tracking, AND use an outer Sync/Heading controller to correct the robot's path.