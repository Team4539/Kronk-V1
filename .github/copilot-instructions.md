# Kronk Robot Code - AI Agent Instructions

## Project Overview

**Kronk** is an FRC robot codebase (Team 4539) for the 2026 season. All shooting intelligence runs **on-board the roboRIO** using pose-based lookup tables — no external coprocessor required.

- **Framework**: WPILib 2026.2.1 (Java 17, Gradle-based)
- **Drivetrain**: CTRE Phoenix 6 Swerve (TunerX-generated in `generated/TunerConstants.java`)
- **Autonomous**: PathPlanner 2026.1.2 with holonomic path following
- **Shooting**: Fixed shooter (no turret) — `ShootingCalculator` uses PhotonVision + pose-based RPM interpolation
- **Controls**: Single Xbox controller for both driving and shooting

## Architecture & Data Flow

### On-Board Shooting System

The shooting system runs entirely on the roboRIO with no external dependencies:

1. **PhotonVision** provides robot pose via AprilTag detection
2. **ShootingCalculator** (singleton, `util/ShootingCalculator.java`) computes:
   - Distance to target (HUB or TRENCH based on `GameStateManager` target mode)
   - Robot-relative angle to target (driver/auto must rotate the robot to aim)
   - Flywheel RPM via inverse-distance-weighted interpolation from `Constants.Shooter.SHOOTING_CALIBRATION`
3. **Update loop**: `RobotContainer.updateVisionPose()` calls `shootingCalculator.update()` every cycle with current pose, chassis speeds, target mode, and RPM offset
4. **Commands read** from `ShootingCalculator` — they never calculate independently

**Key files**: `util/ShootingCalculator.java`, `commands/AutoShootCommand.java`

### Fixed Shooter Design

Since there is **no turret**, aiming is done by rotating the entire robot:
- `ShootingCalculator.getAngleToTarget()` returns how far off the robot heading is from the target
- `ShootingCalculator.getTargetRPM()` returns the interpolated RPM for the current position
- **RPM interpolation**: `List<double[]>` stores calibration points as `{relX, relY, bearingDeg, shooterRPM}`, interpolated using inverse-distance-weighting

### Command Architecture

- **RobotContainer**: Central wiring hub. Subsystems are **conditionally initialized** based on `Constants.SubsystemEnabled` flags (set to `false` to disable for testing). **Single Xbox controller** for all inputs.
- **AutoShootCommand**: Orchestrates shooter spinup + ball feeding. Alliance-aware (only shoots during your alliance's scoring window).
- **Named Commands**: Registered in `RobotContainer.registerNamedCommands()` for PathPlanner auto routines.

### Single Xbox Controller Layout

All controls on one controller (port 0):

| Button | Function |
|--------|----------|
| Left Stick | Drive X/Y (translation) |
| Right Stick X | Rotation |
| RT | Slow mode (proportional) |
| LT | Auto-shoot (hold) |
| RB | Intake deploy (hold) / retract (release) |
| LB | Pre-spool shooter (hold) |
| A | E-stop all motors |
| B | Force shoot toggle |
| X | Shuttle mode toggle |
| Y | Reset gyro |
| Start | Reset game state |
| Back | Toggle field/robot centric |
| POV Up | Point wheels forward (hold) |
| POV Down | Feed shot for calibration (hold) |

### Game State Management

**GameStateManager** (singleton) tracks:
- **Alliance color** (Blue/Red) for field-flipped coordinates
- **Game phases** (AUTO, SHIFT_1-4, END_GAME) based on match timer
- **Target mode**: HUB (scoring) vs TRENCH (shuttling) - can auto-switch based on robot position
- **Active alliance windows**: Only your alliance can score during designated 25-second shifts

**Critical**: All pose-based aiming auto-adjusts for Red alliance (field coordinates are mirrored).

### Calibration System

**CalibrationManager** (singleton) provides SmartDashboard sliders for live-tuning:
- Shooter RPM and RPM offset
- Camera position offsets (via Constants.Vision)
- **Outputs ready-to-paste code** for updating `Constants.java`

**Workflow**: Tune sliders -> Copy generated code from dashboard -> Update `Constants.java`

## Essential Developer Workflows

### Building & Deploying

```bash
# Build robot code
./gradlew build

# Deploy to robot (requires team number in .wpilib/wpilib_preferences.json)
./gradlew deploy

# Simulate robot code (launches Driver Station + GUI)
./gradlew simulateJava
```

**Vendor dependencies** are in `vendordeps/` - Phoenix 6, PathPlanner, PhotonLib, WPILib.

### PathPlanner Autonomous

- **Paths/Autos**: `src/main/deploy/pathplanner/` (edited in PathPlanner GUI app)
- **Configuration**: `pathplanner/settings.json` contains robot dimensions, max velocities, PID constants
- **AutoBuilder**: Configured in `CommandSwerveDrivetrain.configureAutoBuilder()` with alliance flipping enabled
- **Named Commands**: Must be registered in `RobotContainer` before auto chooser is built

## Project-Specific Conventions

### Constants Organization

`Constants.java` has nested static classes by subsystem:
- `SubsystemEnabled` - **Toggle subsystems on/off for testing** (set to `false` to disable)
- `CANIds` - All CAN bus device IDs in one place
- `Field` - Field geometry, alliance-specific poses (BLUE_HUB_CENTER, RED_TRENCH_ROTATING)
- Per-subsystem constants (Shooter, Intake, Trigger, Vision, LEDs, etc.)

**Pattern**: Calibration data uses `List<double[]>` with inverse-distance-weighted interpolation (e.g., `{relX, relY, bearingDeg, shooterRPM}`).

### Subsystem Nullability Pattern

All subsystems in `RobotContainer` are nullable and guarded:

```java
private final ShooterSubsystem shooter;  // Can be null if disabled
// Later:
if (shooter != null) {
    shooter.setTargetRPM(rpm);
}
```

**Why**: Allows testing individual subsystems without full robot hardware.

### Dashboard Organization

Use `DashboardHelper.Category` enum to organize SmartDashboard entries:
- `PRE_MATCH`, `AUTO`, `TELEOP`, `POST_MATCH` - Match phase tabs
- `SETTINGS` - Tunable parameters and calibration commands
- `DEBUG` - Diagnostic info

**Pattern**: `DashboardHelper.putNumber(Category.MATCH, "Shooter/TargetRPM", rpm)`

### Elastic Dashboard Integration

`Elastic.sendNotification()` sends rich notifications to Elastic Dashboard:
- Used in `Robot.java` for mode transitions (Auto started, Teleop started)
- `Elastic.selectTab()` can programmatically switch dashboard tabs
- Notification levels: INFO, WARNING, ERROR

## Critical Gotchas

1. **Alliance flipping**: All field coordinates in `Constants.Field` have BLUE and RED variants. VisionSubsystem auto-flips based on `GameStateManager.getRobotAlliance()`.

2. **Fixed shooter**: There is NO turret. Aiming is done by rotating the whole robot. `ShootingCalculator.getAngleToTarget()` tells you how far off you are.

3. **Phoenix 6 config**: Swerve modules are **generated code** in `generated/TunerConstants.java`. Modify via Phoenix Tuner X, not by hand.

4. **PathPlanner coordinate system**: Paths are drawn for **Blue alliance origin**. AutoBuilder flips for Red automatically.

5. **Voltage compensation**: Shooter RPMs are controlled via CTRE VelocityVoltage PID on the motor controllers.

6. **Single controller**: All driver and operator functions are on ONE Xbox controller (port 0). There is no separate operator controller.

## Key Files Reference

| File | Purpose |
|------|---------|
| `RobotContainer.java` | Subsystem initialization, single-controller bindings, named commands |
| `ShootingCalculator.java` | On-board shooting math: distance, angle, RPM interpolation |
| `GameStateManager.java` | Alliance color, game phases, target mode logic |
| `CalibrationManager.java` | Live tuning sliders, code generation for Constants |
| `AutoShootCommand.java` | Full shooting sequence with alliance window checks |
| `CommandSwerveDrivetrain.java` | Phoenix 6 swerve + PathPlanner integration + field visualization |
| `Constants.java` | All tunable values, subsystem enable flags, shooting calibration tables |
| `VisionSubsystem.java` | PhotonVision camera, AprilTag pose estimation |

## Common Tasks

**Add a new subsystem**:
1. Create subsystem class in `subsystems/`
2. Add CAN IDs to `Constants.CANIds`
3. Add enable flag to `Constants.SubsystemEnabled`
4. Initialize in `RobotContainer` with null-check guard
5. Register named commands if needed for PathPlanner

**Add a calibration parameter**:
1. Add field to `CalibrationManager`
2. Add SmartDashboard slider in `CalibrationManager` constructor
3. Generate code output in `printCalibrationTable()`
4. Update `Constants.java` from dashboard output

**Tune shooting calibration**:
1. Drive robot to known positions relative to the target
2. Use `CalibrationManager` sliders to adjust RPM until shots land
3. Click "RecordPoint" to save `{relX, relY, bearing, RPM}` tuples
4. Click "PrintTable" to export copy-paste code
5. Update `Constants.Shooter.SHOOTING_CALIBRATION` list
6. `ShootingCalculator` will automatically interpolate between calibration points
