# Kronk Calibration Guide -- Team 4539

Complete step-by-step guide for calibrating Kronk's subsystems on real hardware.

> **Prerequisites**: Robot fully assembled, battery charged (12.5V+), roboRIO imaged, PhotonVision configured, all CAN devices flashing green. Connect a laptop with Elastic Dashboard or Shuffleboard.

---

## Table of Contents

1. [Pre-Calibration Checklist](#1-pre-calibration-checklist)
2. [Swerve Drivetrain](#2-swerve-drivetrain)
3. [Intake Pivot](#3-intake-pivot)
4. [PhotonVision Camera](#4-photonvision-camera)
5. [Shooter RPM Tables](#5-shooter-rpm-tables)
6. [RPM Offset Fine-Tuning](#6-rpm-offset-fine-tuning)
7. [Full Shooter Calibration](#7-full-shooter-calibration)
8. [PathPlanner Autonomous](#8-pathplanner-autonomous)
9. [Match Day Quick-Check](#9-match-day-quick-check)

---

## 1. Pre-Calibration Checklist

Before calibrating anything:

- [ ] Battery is **>12.0V** (check `RobotController.getBatteryVoltage()` on dashboard)
- [ ] All CAN devices show green LEDs (no flashing orange/red)
- [ ] Phoenix Tuner X shows all motors + CANcoders + Pigeon2 on the CAN bus
- [ ] PhotonVision is powered and accessible at `http://photonvision.local:5800`
- [ ] Robot code deploys successfully: `./gradlew deploy`
- [ ] Elastic Dashboard or Shuffleboard is connected

### Enabling/Disabling Subsystems for Testing

In `Constants.java` -> `SubsystemEnabled`, toggle subsystems:

```java
public static final boolean DRIVETRAIN = true;
public static final boolean SHOOTER = true;
public static final boolean VISION = true;
public static final boolean TRIGGER = true;
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
   - Slow mode works (RT)
   - Field/robot centric toggle works (Back button)

---

## 3. Intake Pivot

**Why**: The intake pivot angle determines deployed and retracted positions.

**Setup**:
1. Enable `INTAKE = true`
2. Deploy code

**Procedure**:

### CANcoder Offset
1. Manually position the intake to its **retracted/stowed** position
2. Read the raw CANcoder value from `Intake/CANcoderDeg` on dashboard
3. Calculate the offset so that the retracted position reads the `RETRACTED_ANGLE_DEG` value in Constants
4. Update `Constants.Intake.CANCODER_OFFSET_DEG`

### Pivot Angles
1. Check `Constants.Intake` values:
   - `RETRACTED_ANGLE_DEG` -- Stowed position (currently `0.0`)
   - `DEPLOYED_ANGLE_DEG` -- Fully deployed position (currently `130.0`)
   - `IDLE_ANGLE_DEG` -- Idle/ready position (currently `20.0`)
   - `MIN_PIVOT_ANGLE_DEG` -- Minimum safe angle (currently `0.0`)
   - `MAX_PIVOT_ANGLE_DEG` -- Maximum safe angle (currently `130.0`)
2. Deploy and retract the intake using RB on the controller
3. Verify the intake reaches proper positions without binding
4. Adjust angle constants if needed

### Pivot PID
1. Adjust `Constants.Intake.PIVOT_PID_P` (currently `0.02`)
2. Goals:
   - Smooth deploy/retract motion
   - No overshoot past physical limits
   - Holds position when retracted
3. `PIVOT_MAX_OUTPUT` (currently `0.3`) limits maximum motor power

---

## 4. PhotonVision Camera

**Why**: Accurate camera mounting parameters are essential for pose estimation.

**Setup**:
1. Enable `VISION = true`, `DRIVETRAIN = true`
2. Deploy code
3. Ensure AprilTags are visible on the field

**Procedure**:

### Camera Position Calibration
1. Run the `VisionCalibrationCommand` from SmartDashboard (`Tuning/Cal: Vision`)
2. Place the robot at a **known position** on the field (measure precisely)
3. Enter known X/Y in `Cal/vision/KnownX` and `Cal/vision/KnownY`
4. Compare the vision-reported pose with the actual position
5. Click `ValidatePosition` for an accuracy grade
6. If error is high, adjust camera mounting values in `Constants.Vision`:
   - `CAMERA_X_OFFSET` -- Forward/backward from robot center (+ = forward)
   - `CAMERA_Y_OFFSET` -- Left/right from robot center (+ = left)
   - `CAMERA_Z_OFFSET` -- Height from ground to camera lens
   - `CAMERA_PITCH_DEGREES` -- Tilt angle (+ = tilted up)

### Vision Trust
The standard deviations control how much the pose estimator trusts vision vs odometry:

- **Lower values** = trust vision more
- **Higher values** = trust odometry more

Current values in `Constants.Vision`:
- `VISION_STD_DEV_X` = `0.7`
- `VISION_STD_DEV_Y` = `0.7`
- `VISION_STD_DEV_THETA` = `10`

**Test**: Drive around and watch the "Field" visualization. The robot position should update smoothly, stay accurate when seeing tags, and not drift when tags are lost.

---

## 5. Shooter RPM Tables

**Why**: The fixed shooter needs different RPMs at different positions to hit the hub/trench targets.

**Setup**:
1. Enable `SHOOTER = true`, `VISION = true`, `TRIGGER = true`
2. Deploy code
3. Load game pieces into the robot

**Procedure**:

### Starting a Calibration Session
1. Click `Tuning/StartCalibration` on SmartDashboard -- this zeros all offsets for a clean baseline
2. Run `Tuning/Cal: Full Shooter` or `Tuning/Cal: Shooting` command

### Recording Calibration Points
1. Position the robot at a known location where you can see AprilTags
2. Adjust `Tuning/Shooter/RPM` slider until shots consistently score
3. Use **POV Down** on the controller to feed balls into the shooter
4. Click `Tuning/RecordPoint` to save the current position and RPM
5. Move to the next position and repeat
6. **Record at least 10 points** at various positions and angles for good coverage

### Exporting Calibration Data
1. Click `Tuning/PrintTable` to export all recorded points to the console
2. Copy the generated Java code to `Constants.Shooter.SHOOTING_CALIBRATION`
3. Click `Tuning/EndCalibration` to re-enable baked-in offsets

### Calibration Points Format
Each point stores `{relX, relY, bearingDeg, shooterRPM}`:
- `relX` -- Robot X relative to target (meters)
- `relY` -- Robot Y relative to target (meters)
- `bearingDeg` -- Robot-relative angle to target (degrees)
- `shooterRPM` -- Shooter motor RPM at this position

### Tips
- **Start close** and work outward
- Cover a variety of angles, not just straight-on shots
- Charge the battery between sessions (power affects consistency)
- The `Cal/Status` display shows current position and distance info

---

## 6. RPM Offset Fine-Tuning

**Why**: After building calibration tables, you may find systematic errors (all shots slightly short/long).

**Setup**:
1. All shooting subsystems enabled
2. Deploy code

**Procedure**:
1. Run the `Tuning/Cal: Distance Offset` command
2. Shoot at multiple positions and observe patterns:
   - **Shots consistently short** -> Increase `Tuning/Shooter/RPMOffset`
   - **Shots consistently long** -> Decrease `Tuning/Shooter/RPMOffset`
3. The RPM offset is added to all calculated RPMs globally

---

## 7. Full Shooter Calibration

**Why**: End-to-end testing of the complete shooting system.

**Procedure**:
1. Run `Tuning/Cal: Full Shooter`
2. The display shows: distance, relative X/Y, bearing, and current RPM
3. Adjust `Tuning/Shooter/RPM` to tune shots
4. Use **POV Down** on controller to feed balls
5. Click `Tuning/RecordPoint` after a good shot
6. Click `Tuning/PrintTable` when done to get copy-paste code

**Remember**: Since this is a fixed shooter, you must **rotate the entire robot** to aim at the target. The `Cal/Status` display shows the bearing angle to help you orient correctly.

---

## 8. PathPlanner Autonomous

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
   - `TrainingShot` -- Training shot (2.5s timeout)
   - `spinUpShooter` -- Pre-spool shooter
   - `stopShooter` -- Stop shooter motors
   - `enableShuttleMode` / `disableShuttleMode` -- Toggle trench mode
   - `wait0.5` / `wait1.0` / `wait2.0` -- Timing delays

### Testing
1. Use PathPlanner's built-in simulation
2. Test on robot in `simulateJava` mode: `./gradlew simulateJava`
3. On real robot: Select auto from SmartDashboard chooser, run in auto mode
4. Watch the "Field" visualization for path tracking accuracy

---

## 9. Match Day Quick-Check

Before each match:

### Pre-Match (in pit)
- [ ] Battery is **>12.5V**
- [ ] All CAN devices show green LEDs
- [ ] Intake deploys and retracts smoothly
- [ ] Shooter wheels spin freely (no binding)
- [ ] PhotonVision powered and showing camera feed

### Pre-Match (on field)
- [ ] Connect laptop and deploy latest code
- [ ] Dashboard shows correct alliance color
- [ ] Vision/HasTarget = `true` when pointing at tags
- [ ] Select autonomous routine from chooser
- [ ] LEDs show alliance color correctly

### Post-Match
- [ ] Check battery voltage (if <11V, swap battery)
- [ ] Check motor temperatures on dashboard
- [ ] Note any issues for pit crew

---

## Constants Quick Reference

| Constant | Location | Default | Purpose |
|----------|----------|---------|---------|
| `SHOOTING_CALIBRATION` | `Shooter` | List<double[]> | Pose-based RPM calibration |
| `DEFAULT_IDLE_RPM` | `Shooter` | `500.0` | Idle flywheel RPM |
| `CALIBRATION_BEARING_WEIGHT` | `Shooter` | `0.05` | Bearing weight in interpolation |
| `RETRACTED_ANGLE_DEG` | `Intake` | `0.0` | Intake retracted angle |
| `DEPLOYED_ANGLE_DEG` | `Intake` | `130.0` | Intake deployed angle |
| `CANCODER_OFFSET_DEG` | `Intake` | `-100.107` | CANcoder zero offset |
| `PIVOT_PID_P` | `Intake` | `0.02` | Intake pivot PID |
| `CAMERA_X_OFFSET` | `Vision` | `0.2032` | Camera forward offset |
| `CAMERA_Y_OFFSET` | `Vision` | `-0.1524` | Camera lateral offset |
| `CAMERA_Z_OFFSET` | `Vision` | `0.23495` | Camera height |
| `CAMERA_PITCH_DEGREES` | `Vision` | `38` | Camera tilt angle |
| `VISION_STD_DEV_X/Y` | `Vision` | `0.7` | Vision trust X/Y |
| `VISION_STD_DEV_THETA` | `Vision` | `10` | Vision trust rotation |

---

*Last updated: March 2026 -- Team 4539 Kronk (Fixed Shooter)*
