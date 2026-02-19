# Kronk Calibration Guide -- Team 4539

Complete step-by-step guide for calibrating Kronk's subsystems on real hardware.

> **Prerequisites**: Robot fully assembled, battery charged (12.5V+), roboRIO imaged, Limelight configured, all CAN devices flashing green. Connect a laptop with Elastic Dashboard or Shuffleboard.

---

## Table of Contents

1. [Pre-Calibration Checklist](#1-pre-calibration-checklist)
2. [Swerve Drivetrain](#2-swerve-drivetrain)
3. [Turret Gear Ratio](#3-turret-gear-ratio)
4. [Turret PID Tuning](#4-turret-pid-tuning)
5. [Turret Per-Tag Angle Offsets](#5-turret-per-tag-angle-offsets)
6. [Intake Pivot](#6-intake-pivot)
7. [Limelight Vision](#7-limelight-vision)
8. [Shooter Power Tables](#8-shooter-power-tables)
9. [Distance Offset Fine-Tuning](#9-distance-offset-fine-tuning)
10. [Full Shooter + Turret Calibration](#10-full-shooter--turret-calibration)
11. [Raspberry Pi Coprocessor](#11-raspberry-pi-coprocessor)
12. [PathPlanner Autonomous](#12-pathplanner-autonomous)
13. [Match Day Quick-Check](#13-match-day-quick-check)

---

## 1. Pre-Calibration Checklist

Before calibrating anything:

- [ ] Battery is **>12.0V** (check `RobotController.getBatteryVoltage()` on dashboard)
- [ ] All CAN devices show green LEDs (no flashing orange/red)
- [ ] Phoenix Tuner X shows all motors + CANcoders + Pigeon2 on the CAN bus
- [ ] Limelight is powered and accessible at `http://limelight.local:5801`
- [ ] Robot code deploys successfully: `.\gradlew deploy`
- [ ] Elastic Dashboard or Shuffleboard is connected

### Enabling/Disabling Subsystems for Testing

In `Constants.java` -> `SubsystemEnabled`, toggle subsystems:

```java
public static final boolean DRIVETRAIN = true;   // Swerve drive
public static final boolean TURRET = false;       // Set true when calibrating turret
public static final boolean SHOOTER = false;      // Set true when calibrating shooter
public static final boolean LIMELIGHT = false;    // Set true when calibrating vision
public static final boolean SPINDEXER = false;
public static final boolean TURRET_FEED = false;
public static final boolean INTAKE = true;
public static final boolean LEDS = true;
```

> **Tip**: Only enable subsystems you're actively calibrating. This prevents unexpected motor movement.

---

## 2. Swerve Drivetrain

The swerve modules are **generated code** from Phoenix Tuner X. Do NOT edit `generated/TunerConstants.java` by hand.

### CANcoder Offsets

1. Open **Phoenix Tuner X**
2. Select each swerve module's CANcoder
3. Point all wheels **straight forward** (use a straight edge)
4. Record the absolute position for each module
5. Enter these values in Tuner X's device configuration
6. Re-export `TunerConstants.java`

### Pigeon2 IMU

1. Place robot on a flat, level surface
2. In Phoenix Tuner X, zero the Pigeon2 yaw
3. Verify heading reads 0 deg when facing forward on the field

### Drive Testing

1. Enable `DRIVETRAIN = true` in Constants
2. Deploy and enable in teleop
3. Verify:
   - All 4 wheels spin in the correct direction
   - Robot drives forward when pushing left stick forward
   - Robot rotates counterclockwise when pushing right stick left
   - Slow mode works (left bumper)
   - Robot-centric toggle works (right bumper)

---

## 3. Turret Gear Ratio

**Why**: The turret gear ratio converts motor rotations to turret degrees. An incorrect ratio means the turret will over- or under-shoot angle commands.

**Setup**:
1. Enable `TURRET = true` in Constants
2. Deploy code

**Procedure**:
1. **Mark the turret's current position** with tape on both the turret and the frame
2. Run the `CalibrateTurretGearRatio` command from SmartDashboard
3. **Manually rotate the turret exactly 360 deg** by hand (back to the tape mark)
4. Cancel the command
5. Read `Calibration/CalculatedGearRatio` from SmartDashboard
6. Update `Constants.Turret.GEAR_RATIO` with this value

**Current value**: `10.00537109375`

> **Important**: The turret has a 270 deg range centered at 0 deg (-135 deg to +135 deg). Do NOT rotate past the hardstops during this calibration.

---

## 4. Turret PID Tuning

**Why**: PID gains control how smoothly and accurately the turret tracks target angles.

**Setup**:
1. Enable `TURRET = true`
2. Deploy code

**Procedure**:
1. Run the `TurretPIDCalibration` command
2. On SmartDashboard, adjust:
   - `Cal/Turret/PID_P` -- Proportional gain (start at `0.2`)
   - `Cal/Turret/PID_I` -- Integral gain (start at `0.0`)
   - `Cal/Turret/PID_D` -- Derivative gain (start at `0.01`)
   - `Cal/Turret/TargetAngle` -- Command angle to test response
3. **Tuning goals**:
   - Turret reaches target angle quickly (< 0.5s for 90 deg)
   - Minimal overshoot (< 2 deg)
   - No oscillation at rest
   - Smooth movement, no jerking
4. Update `Constants.Turret.PID_P/I/D` with final values

**Current values**: P=`0.2`, I=`0.0`, D=`0.01`

### Turret Angle Range

The turret operates from **-135 deg to +135 deg** (270 deg total range):
- `0 deg` = turret facing forward (centered)
- `+90 deg` = turret rotated 90 deg counterclockwise (from top view)
- `-90 deg` = turret rotated 90 deg clockwise
- Warning at +/-125 deg (10 deg from limits)

---

## 5. Turret Per-Tag Angle Offsets

**Why**: Different AprilTags may have slight mounting variations. Per-tag offsets correct for this.

**Setup**:
1. Enable `TURRET = true`, `LIMELIGHT = true`
2. Deploy code

**Procedure**:
1. Run the `FullShooterCalibration` command
2. Drive to where you can see one AprilTag clearly
3. Use `Cal/Turret/ManualAngle` to aim the turret at the target
4. Note the difference between the calculated angle and the angle that actually hits
5. Record the offset on SmartDashboard: `Offset/Angle_TagXX`
6. Repeat for each visible tag
7. Toggle `Cal/RecordOffset` to save the offset
8. Copy the generated Java code from `Cal/GeneratedCode` to `Constants.Turret.TAG_ANGLE_OFFSETS`

---

## 6. Intake Pivot

**Why**: The intake pivot angle determines deployed and retracted positions.

**Setup**:
1. Enable `INTAKE = true`
2. Deploy code

**Procedure**:

### CANcoder Offset
1. Manually position the intake to its **retracted/stowed** position
2. Read the raw CANcoder value from `Intake/CANcoderDeg` on dashboard
3. Calculate the offset so that the retracted position reads the `IDLE_ANGLE_DEG` value in Constants
4. Update `Constants.Intake.CANCODER_OFFSET_DEG`

### Pivot Angles
1. Check `Constants.Intake` values:
   - `IDLE_ANGLE_DEG` -- Retracted/stowed position (currently `0.0`)
   - `DEPLOYED_ANGLE_DEG` -- Fully deployed position (currently `90.0`)
   - `MIN_PIVOT_ANGLE_DEG` -- Minimum safe angle (currently `-5.0`)
   - `MAX_PIVOT_ANGLE_DEG` -- Maximum safe angle (currently `105.0`)
2. Deploy and retract the intake using commands
3. Verify the intake reaches proper positions without binding
4. If the intake doesn't reach far enough or goes too far, adjust the angle constants

### Pivot PID
1. Adjust `Constants.Intake.PIVOT_PID_P` (currently `0.008`)
2. Goals:
   - Smooth deploy/retract motion
   - No overshoot past physical limits
   - Holds position when retracted
3. `PIVOT_MAX_OUTPUT` (currently `0.2`) limits maximum motor power -- increase if too slow, decrease if too aggressive

---

## 7. Limelight Vision

**Why**: Accurate camera mounting parameters are essential for pose estimation.

**Setup**:
1. Enable `LIMELIGHT = true`, `DRIVETRAIN = true`
2. Deploy code
3. Ensure AprilTags are visible on the field

**Procedure**:

### Camera Position Calibration
1. Run the `LimelightCalibration` command
2. Place the robot at a **known position** on the field (measure precisely)
3. Compare the Limelight-reported pose (`Limelight/HasPose`) with the actual position
4. Adjust `Cal/Limelight/CameraOffsetX/Y/Z` until the reported pose matches reality
5. Adjust `Cal/Limelight/CameraPitch` if the vertical angle is off
6. Copy values to `Constants.Limelight`

### Vision Trust Calibration
The vision standard deviations control how much the pose estimator trusts vision vs odometry:

- **Lower values** = trust vision more (good when close to tags, bad when far)
- **Higher values** = trust odometry more

Current values in `Constants.Limelight`:
- `VISION_STD_DEV_X` / `Y` / `THETA`

**Test**: Drive the robot around and watch the "Field" visualization. The robot position should:
- Update smoothly (no jumping)
- Stay accurate when seeing tags
- Not drift when tags are lost (odometry takes over)

---

## 8. Shooter Power Tables

**Why**: The shooter needs different motor powers at different distances to hit the hub/trench.

**Setup**:
1. Enable `TURRET = true`, `SHOOTER = true`, `LIMELIGHT = true`
2. Deploy code
3. Load game pieces into the robot

**Procedure**:

### Hub Shooting Calibration
1. Run the `ShootingCalibration` command
2. Position the robot at a known distance from the hub (start at **2m**)
3. Adjust `Calibration/TopPower` and `Calibration/BottomPower` on SmartDashboard
   - **Top motor** controls arc/height (more power = higher trajectory)
   - **Bottom motor** controls distance (more power = farther shot)
4. Shoot game pieces and adjust until they consistently score
5. Toggle `Calibration/RecordShot` to log the calibration point
6. Move to the next distance (**3m, 4m, 5m**, etc.) and repeat
7. Copy the generated `put(distance, new double[]{top, bottom})` lines to `Constants.Shooter.SHOOTING_CALIBRATION`

### Trench Shuttling Calibration
1. Repeat the process but aim at the trench target
2. Trench shots typically need a **flatter trajectory** (less top motor power)
3. Copy results to `Constants.Shooter.TRENCH_CALIBRATION`

### Calibration Tips
- **Start close** (2m) and work outward
- Record at least **5 different distances** for smooth interpolation
- Charge the battery between calibration sessions (power affects shot consistency)
- The voltage compensation system (on the Pi) accounts for battery droop, but start with a full battery

---

## 9. Distance Offset Fine-Tuning

**Why**: After building calibration tables, you may find systematic errors (all shots slightly short, etc.).

**Setup**:
1. All shooting subsystems enabled
2. Deploy code

**Procedure**:
1. Run the `DistanceOffsetCalibration` command
2. Shoot at multiple distances and observe patterns:
   - **Shots consistently short** -> Increase `Constants.Shooter.BOTTOM_MOTOR_POWER_OFFSET`
   - **Shots consistently high** -> Decrease `Constants.Shooter.TOP_MOTOR_POWER_OFFSET`
   - **Shots consistently long** -> Decrease bottom motor offset
3. Adjust via SmartDashboard: `Cal/Shooter/TopPowerOffset` and `Cal/Shooter/BottomPowerOffset`
4. Copy final values to `Constants.Shooter.TOP_MOTOR_POWER_OFFSET` and `BOTTOM_MOTOR_POWER_OFFSET`

---

## 10. Full Shooter + Turret Calibration

**Why**: The `FullShooterCalibration` command combines turret aiming and shooter control for end-to-end testing.

**Procedure**:
1. Run `FullShooterCalibration`
2. Set `Cal/Turret/UseManual = true` to manually aim
3. Adjust `Cal/Turret/ManualAngle` to point at target
4. Adjust `Cal/Shooter/TopPower` and `Cal/Shooter/BottomPower`
5. Fire and observe results
6. Use `Cal/Turret/AngleOffset` to fine-tune aim for specific tags
7. Toggle `Cal/RecordOffset` to save per-tag offsets
8. The generated Java code appears in `Cal/GeneratedCode` -- copy to Constants

---

## 11. Raspberry Pi Coprocessor

The Pi runs `pi_shooting.py` which calculates the full shooting solution using a trained ML model.

### Initial Setup

1. **Connect Pi to robot network** (10.45.39.x subnet)
2. Install dependencies on Pi:
   ```bash
   pip3 install -r requirements.txt
   ```
3. Test NetworkTables connection:
   ```bash
   python3 test_connection.py
   ```
4. Verify connection: `Pi/Status/Connected` should show `true` on SmartDashboard

### Training Data Collection

Training data comes from calibration shots. The Pi **automatically creates** a CSV file at `shooting_training_data.csv` when shots are recorded.

1. Shoot from various positions, distances, and while moving
2. Mark hits/misses via NetworkTables (`Pi/Training/RecordHit`, `Pi/Training/RecordMiss`)
3. More data = better ML model predictions

### Training the Model

```bash
# On the Pi:
python3 train_model.py

# Or trigger remotely via NetworkTables:
# Set Pi/Training/RequestRetrain = true
```

The Pi auto-retrains whenever:
- The CSV file is modified
- `Pi/Training/RequestRetrain` is set to `true`
- On startup if training data exists

### Verifying Pi Operation

On SmartDashboard, check:
- `Pi/Connected` = `true`
- `Pi/UsingFallback` = `false`
- `Pi/Status/model_loaded` = `true` (if model has been trained)
- `Pi/Status/loop_time_ms` < 20ms (healthy update rate)

### If Pi Disconnects

The robot automatically falls back to local calibration tables (`Constants.Shooter.SHOOTING_CALIBRATION`). The fallback:
- Uses direct angle-to-target calculation (no ML corrections)
- Interpolates shooter powers from the calibration TreeMap
- No lead compensation for moving shots
- Confidence drops to 0.3 (vs 0.9+ with ML model)

See `pi/TROUBLESHOOTING.md` for detailed Pi debugging steps.

---

## 12. PathPlanner Autonomous

### Setup
1. Open PathPlanner app (separate download)
2. Paths are in `src/main/deploy/pathplanner/`
3. Robot dimensions and max velocities are in `pathplanner/settings.json`

### Creating Auto Routines
1. Draw paths in the PathPlanner GUI
2. All paths are drawn for **Blue alliance origin** -- Red flipping is automatic
3. Use named commands in auto sequences:
   - `autoShoot` -- Full shoot sequence (3s timeout)
   - `quickShoot` -- Fast shoot (1.5s timeout)
   - `aimAtPose` -- Pre-aim turret (2s timeout)
   - `spinUpShooter` -- Pre-spool shooter
   - `stopShooter` -- Stop shooter motors
   - `centerTurret` -- Return turret to 0 deg
   - `enableShuttleMode` / `disableShuttleMode` -- Toggle trench mode

### Testing
1. Use PathPlanner's built-in simulation
2. Test on robot in `simulateJava` mode: `.\gradlew simulateJava`
3. On real robot: Select auto from SmartDashboard chooser, run in auto mode
4. Watch the "Field" visualization for path tracking accuracy

---

## 13. Match Day Quick-Check

Before each match:

### Pre-Match (in pit)
- [ ] Battery is **>12.5V**
- [ ] All CAN devices show green LEDs
- [ ] Turret moves freely through full range (-135 deg to +135 deg)
- [ ] Intake deploys and retracts smoothly
- [ ] Shooter wheels spin freely (no binding)
- [ ] Limelight powered and showing camera feed

### Pre-Match (on field)
- [ ] Connect laptop and deploy latest code
- [ ] Dashboard shows correct alliance color
- [ ] `Pi/Connected` = `true` (if using Pi)
- [ ] `Limelight/HasTarget` = `true` when pointing at tags
- [ ] Select autonomous routine from chooser
- [ ] LEDs show **no auto selected** warning clears after selecting auto
- [ ] LEDs show **FMS disconnected** warning clears when FMS connects

### Post-Match
- [ ] Check battery voltage (if <11V, swap battery)
- [ ] Check motor temperatures on dashboard
- [ ] Note any issues for pit crew

---

## Constants Quick Reference

| Constant | Location | Default | Purpose |
|----------|----------|---------|---------|
| `GEAR_RATIO` | `Turret` | `10.005` | Motor-to-turret gear ratio |
| `PID_P/I/D` | `Turret` | `0.2/0.0/0.01` | Turret position PID |
| `MIN_ANGLE_DEG` | `Turret` | `-135.0` | Turret min angle |
| `MAX_ANGLE_DEG` | `Turret` | `+135.0` | Turret max angle |
| `ON_TARGET_TOLERANCE_DEG` | `Turret` | `2.0` | Aim accuracy threshold |
| `SHOOTING_CALIBRATION` | `Shooter` | TreeMap | Distance -> [top, bottom] |
| `TRENCH_CALIBRATION` | `Shooter` | TreeMap | Distance -> [top, bottom] |
| `TOP_MOTOR_POWER_OFFSET` | `Shooter` | `0.0` | Global top power adjustment |
| `BOTTOM_MOTOR_POWER_OFFSET` | `Shooter` | `0.0` | Global bottom power adjustment |
| `SPIN_UP_TIME_SECONDS` | `Shooter` | -- | Time to reach full speed |
| `IDLE_ANGLE_DEG` | `Intake` | `0.0` | Intake retracted angle |
| `DEPLOYED_ANGLE_DEG` | `Intake` | `90.0` | Intake deployed angle |
| `CANCODER_OFFSET_DEG` | `Intake` | -- | CANcoder zero offset |
| `PIVOT_PID_P` | `Intake` | `0.008` | Intake pivot PID |
| `VISION_STD_DEV_X/Y/THETA` | `Limelight` | -- | Vision trust levels |

---

*Last updated: February 2026 -- Team 4539 Kronk*
