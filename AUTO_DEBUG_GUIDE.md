# Autonomous Tuning & Debugging Guide

This guide details the specific areas to check when debugging "wrong way" driving, oscillation, or auto-aim issues on the practice field.

## 1. Pre-Flight Verification (Do This First)

Before running any auto, verify the robot understands "Forward".

1.  **Place Robot on Field**: Face the **Blue Alliance Wall** (your back to the driver station if you are Blue).
2.  **Reset Gyro**: Press `Y` on the controller.
3.  **Check Dashboard**:
    *   The Field 2D widget must show the robot facing **0 degrees** (Right arrow on the widget).
    *   If it points 180° (Left), you need to change `kPigeonMountPoseYawDeg` in `src/main/java/frc/robot/generated/TunerConstants.java`.

## 2. Diagnosing "Driving the Wrong Way"

If the robot executes the path shape correctly but in the wrong direction (e.g., drives backwards when it should go forwards, or strafes left instead of right):

### Step A: Use the Debug Toggles
I have added SmartDashboard toggles to test fixes without redeploying code.
*   Look for the **Debug** tab or keys in SmartDashboard.
*   **`Debug/PathPlanner/FlipVx`**: Flips Forward/Backward commands.
    *   *Try this if the robot drives backwards.*
*   **`Debug/PathPlanner/FlipVy`**: Flips Left/Right commands.
    *   *Try this if the robot strafes the wrong way.*

### Step B: Permanent Fix
If checking `FlipVx` makes the auto work:
1.  Open `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`.
2.  Find `transformPathplannerSpeeds`.
3.  Uncomment or add `vx = -vx;`.

## 3. Fixing PathPlanner Geometry (Applied)

We updated the code to match your **27" x 28" Frame** with 4-inch bumpers.

**File**: `src/main/deploy/pathplanner/settings.json`
*   **Robot Size**: 0.889m x 0.9144m (35" x 36" with bumpers)
*   **Module X**: 0.2921m (11.5 inches forward)
*   **Module Y**: 0.2794m (11.0 inches left)

*Calculated assuming modules are centered 2.5 inches from the frame edge.*

## 4. Tuning PID (Oscillation / Shaking)

If the robot "jitters" or shakes violently during auto:

**File**: `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java` (Line ~330)

We lowered the constants to play it safe:
```java
// Translation (X/Y)
new PIDConstants(5.0, 0, 0) // Was 10.0

// Rotation (Theta)
new PIDConstants(5.0, 0, 0.0) // Was 7.0, D=0.5
```

**Tuning Procedure**:
1.  If the robot is **sluggish** or doesn't reach the target: **Increase P** (e.g., 5.0 -> 6.0).
2.  If the robot **oscillates** (wiggles back and forth): **Decrease P** (e.g., 5.0 -> 4.0).
3.  Only add **D** (Start small: 0.1) if the robot moves fast but overshoots the end point.

## 5. Auto-Aim Debugging (Spinning)

If `aimAtPose` or Auto-Aim makes the robot spin continuously or in the wrong direction:

**File**: `src/main/java/frc/robot/Constants.java`
*   Look for `AIM_DIRECTION` in `class Driver`.
*   Currently set to **1.0**.
*   **Fix**: If it spins away from the target, change to **-1.0**.

**Logic Update (RobotContainer.java)**:
*   I removed a hardcoded `* -1.0` in `aimOmega`.
*   Now, positive error should produce positive correction. If your gyro is inverted (CCW is negative), you might need to put the negative sign back or use `AIM_DIRECTION = -1.0`.

## 6. Critical Warnings

1.  **Do NOT use `aimAtPose` inside a moving path**.
    *   The `aimAtPose` command sets Velocity X/Y to **0**.
    *   If you trigger this command *while* the path is running, the robot will slam on the brakes to aim.
    *   **Solution**: Use PathPlanner's **"Rotation Targets"** feature in the GUI to aim while moving. Use `aimAtPose` only for stationary shots.

2.  **Red vs Blue**:
    *   PathPlanner autos are built on the **Blue** side.
    *   The code automatically flips for Red.
    *   **Test on Blue first** to eliminate flipping logic as a variable.
