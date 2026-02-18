# Kronk Robot Code - AI Agent Instructions

## Project Overview

**Kronk** is an FRC robot codebase (Team 4539) for the 2026 season. All shooting intelligence runs **on-board the roboRIO** using distance-based lookup tables and lead angle compensation — no external coprocessor required.

- **Framework**: WPILib 2026.2.1 (Java 17, Gradle-based)
- **Drivetrain**: CTRE Phoenix 6 Swerve (TunerX-generated in `generated/TunerConstants.java`)
- **Autonomous**: PathPlanner 2026.1.2 with holonomic path following
- **Shooting**: On-board `ShootingCalculator` using Limelight vision + distance interpolation tables

## Architecture & Data Flow

### On-Board Shooting System

The shooting system runs entirely on the roboRIO with no external dependencies:

1. **Limelight** provides robot pose via AprilTag detection (`botpose_wpiblue`)
2. **ShootingCalculator** (singleton, `util/ShootingCalculator.java`) computes:
   - Distance to target (HUB or TRENCH based on `GameStateManager` target mode)
   - Robot-relative turret angle to face the target
   - **Lead angle compensation** for shooting while moving (projects perpendicular velocity component)
   - Top/bottom flywheel RPM via interpolation from `Constants.Shooter.SHOOTING_CALIBRATION` / `TRENCH_CALIBRATION`
3. **Update loop**: `RobotContainer.updateVisionPose()` calls `shootingCalculator.update()` every cycle with current pose, chassis speeds, target mode, and turret angle offset
4. **Commands read** from `ShootingCalculator` — they never calculate independently

**Key files**: `util/ShootingCalculator.java`, `commands/AutoShootCommand.java`, `commands/turret/AimTurretToPoseCommand.java`

### Shooting-on-the-Fly Details

`ShootingCalculator` compensates for robot motion:
- **Lead angle**: Based on perpendicular velocity component relative to the target direction
- **Flight time estimation**: `BASE_FLIGHT_TIME_SECONDS (0.5s) + FLIGHT_TIME_PER_METER (0.08s) × distance`
- **Moving threshold**: Lead compensation only applied when robot speed > `MOVING_THRESHOLD_MPS (0.3 m/s)`
- **RPM interpolation**: `TreeMap<Double, double[]>` maps distance → `[topRPM, bottomRPM]`, linearly interpolated

### Command Architecture

- **RobotContainer**: Central wiring hub. Subsystems are **conditionally initialized** based on `Constants.SubsystemEnabled` flags (set to `false` to disable for testing).
- **AutoShootCommand**: Orchestrates turret aiming + shooter spinup + ball feeding. Alliance-aware (only shoots during your alliance's scoring window).
- **AimTurretToPoseCommand**: Default turret command — continuously reads from `ShootingCalculator` to track the target.
- **Named Commands**: Registered in `RobotContainer.registerNamedCommands()` for PathPlanner auto routines.

### Game State Management

**GameStateManager** (singleton) tracks:
- **Alliance color** (Blue/Red) for field-flipped coordinates
- **Game phases** (AUTO, SHIFT_1-4, END_GAME) based on match timer
- **Target mode**: HUB (scoring) vs TRENCH (shuttling) - can auto-switch based on robot position
- **Active alliance windows**: Only your alliance can score during designated 25-second shifts

**Critical**: All pose-based aiming auto-adjusts for Red alliance (field coordinates are mirrored).

### Calibration System

**CalibrationManager** (singleton) provides SmartDashboard sliders for live-tuning:
- Turret PID, angle offsets, gear ratio
- Shooter power offsets, manual power overrides
- Limelight camera position offsets
- **Outputs ready-to-paste code** for updating `Constants.java`

**Workflow**: Tune sliders -> Copy generated code from dashboard -> Update `Constants.java`

## Essential Developer Workflows

### Building & Deploying

```powershell
# Build robot code
.\gradlew build

# Deploy to robot (requires team number in .wpilib/wpilib_preferences.json)
.\gradlew deploy

# Simulate robot code (launches Driver Station + GUI)
.\gradlew simulateJava
```

**Vendor dependencies** are in `vendordeps/` - Phoenix 6, PathPlanner, WPILib.

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
- Per-subsystem constants (Turret, Shooter, Intake, etc.)

**Pattern**: Calibration data uses `TreeMap<Double, double[]>` for interpolation (e.g., distance -> [top_power, bottom_power]).

### Subsystem Nullability Pattern

All subsystems in `RobotContainer` are nullable and guarded:

```java
private final TurretSubsystem turret;  // Can be null if disabled
// Later:
if (turret != null) {
    turret.setTargetAngle(angle);
}
```

**Why**: Allows testing individual subsystems without full robot hardware.

### Dashboard Organization

Use `DashboardHelper.Category` enum to organize SmartDashboard entries:
- `PRE_MATCH`, `AUTO`, `TELEOP`, `POST_MATCH` - Match phase tabs
- `SETTINGS` - Tunable parameters and calibration commands
- `DEBUG` - Diagnostic info

**Pattern**: `DashboardHelper.putNumber(Category.SETTINGS, "Turret/AngleOffset", 0.0)`

### Elastic Dashboard Integration

`Elastic.sendNotification()` sends rich notifications to Elastic Dashboard:
- Used in `Robot.java` for mode transitions (Auto started, Teleop started)
- `Elastic.selectTab()` can programmatically switch dashboard tabs
- Notification levels: INFO, WARNING, ERROR

## Critical Gotchas

1. **Alliance flipping**: All field coordinates in `Constants.Field` have BLUE and RED variants. LimelightSubsystem auto-flips based on `GameStateManager.getRobotAlliance()`.

2. **Turret angle limits**: 270 deg rotation range centered at 0 deg (-135 deg to +135 deg) with 10 deg safety margins. **Never** command angles outside this range or turret will fault.

3. **Phoenix 6 config**: Swerve modules are **generated code** in `generated/TunerConstants.java`. Modify via Phoenix Tuner X, not by hand.

4. **PathPlanner coordinate system**: Paths are drawn for **Blue alliance origin**. AutoBuilder flips for Red automatically.

5. **Voltage compensation**: Shooter RPMs are controlled via CTRE VelocityVoltage PID on the motor controllers.

## Key Files Reference

| File | Purpose |
|------|---------|
| `RobotContainer.java` | Subsystem initialization, button bindings, named commands |
| `ShootingCalculator.java` | On-board shooting math: distance, angle, lead compensation, RPM interpolation |
| `GameStateManager.java` | Alliance color, game phases, target mode logic |
| `CalibrationManager.java` | Live tuning sliders, code generation for Constants |
| `AutoShootCommand.java` | Full shooting sequence with alliance window checks |
| `AimTurretToPoseCommand.java` | Default turret command — continuously tracks target via ShootingCalculator |
| `CommandSwerveDrivetrain.java` | Phoenix 6 swerve + PathPlanner integration |
| `Constants.java` | All tunable values, subsystem enable flags, shooting calibration tables |

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
3. Generate code output in `generateCalibrationCode()`
4. Update `Constants.java` from dashboard output

**Tune shooting calibration**:
1. Drive robot to known distances from the target
2. Use `CalibrationManager` sliders to adjust top/bottom RPM until shots are accurate
3. Record distance → [topRPM, bottomRPM] pairs
4. Update `Constants.Shooter.SHOOTING_CALIBRATION` or `TRENCH_CALIBRATION` TreeMaps
5. `ShootingCalculator` will automatically interpolate between calibration points
