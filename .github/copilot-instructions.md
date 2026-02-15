# Kronk Robot Code - AI Agent Instructions

## Project Overview

**Kronk** is an FRC robot codebase (Team 4539) for the 2026 season. The architecture features a **Raspberry Pi coprocessor** that handles all turret/shooter intelligence using machine learning, with the roboRIO handling subsystems and command execution.

- **Framework**: WPILib 2026.2.1 (Java 17, Gradle-based)
- **Drivetrain**: CTRE Phoenix 6 Swerve (TunerX-generated in `generated/TunerConstants.java`)
- **Autonomous**: PathPlanner 2026.1.2 with holonomic path following
- **Coprocessor**: Raspberry Pi running Python ML model for shooting calculations

## Architecture & Data Flow

### Robot-Pi Communication via NetworkTables

The **core innovation** is offloading shooting intelligence to a Raspberry Pi:

1. **roboRIO → Pi** (`Pi/Input` table): Robot publishes pose, velocity, alliance, target mode, battery voltage
2. **Pi calculates** optimal turret angle, shooter top/bottom speeds using:
   - Trained ML model (`pi/shooting_model.pkl`) if available
   - Interpolation fallback from `Constants.Shooter.SHOOTING_CALIBRATION` if Pi disconnects
3. **Pi → roboRIO** (`Pi/Output` table): Shooting solution consumed by `PiShootingHelper`
4. **Fallback**: If Pi heartbeat timeout (500ms), robot uses local calibration tables

**Key files**: `util/PiShootingHelper.java`, `pi/pi_shooting.py`, `pi/train_model.py`

### Command Architecture

- **RobotContainer**: Central wiring hub. Subsystems are **conditionally initialized** based on `Constants.SubsystemEnabled` flags (set to `false` to disable for testing).
- **AutoShootCommand**: Orchestrates turret aiming + shooter spinup + ball feeding. Alliance-aware (only shoots during your alliance's scoring window).
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

**Workflow**: Tune sliders → Copy generated code from dashboard → Update `Constants.java`

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

### Pi Coprocessor Setup

1. **Deploy CSV**: Training data in `src/main/deploy/shooting_training_data.csv` auto-deploys to roboRIO
2. **Train model**: Run `python3 train_model.py` on Pi (looks for CSV in `/home/lvuser/deploy/` or local dir)
3. **Auto-retrain**: Set `Pi/Training/RequestRetrain` NetworkTables boolean to trigger retraining
4. **Connection check**: Run `python3 test_connection.py` on Pi to diagnose NetworkTables issues

**Important**: Pi must be on robot network (10.45.39.x for Team 4539). Model training is automatic on startup if CSV exists.

## Project-Specific Conventions

### Constants Organization

`Constants.java` has nested static classes by subsystem:
- `SubsystemEnabled` - **Toggle subsystems on/off for testing** (set to `false` to disable)
- `CANIds` - All CAN bus device IDs in one place
- `Field` - Field geometry, alliance-specific poses (BLUE_HUB_CENTER, RED_TRENCH_ROTATING)
- Per-subsystem constants (Turret, Shooter, Intake, etc.)

**Pattern**: Calibration data uses `TreeMap<Double, double[]>` for interpolation (e.g., distance → [top_power, bottom_power]).

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

2. **Turret angle limits**: 270° rotation range centered at 0° (-135° to +135°) with 10° safety margins. **Never** command angles outside this range or turret will fault.

3. **Phoenix 6 config**: Swerve modules are **generated code** in `generated/TunerConstants.java`. Modify via Phoenix Tuner X, not by hand.

4. **PathPlanner coordinate system**: Paths are drawn for **Blue alliance origin**. AutoBuilder flips for Red automatically.

5. **Voltage compensation**: Shooter powers are voltage-normalized on Pi. Battery voltage is published via NetworkTables for accurate compensation.

6. **Heartbeat monitoring**: `PiShootingHelper` checks Pi heartbeat every 500ms. If timeout, falls back to local calibration tables. Check `Pi/Status/Connected` to diagnose.

## Key Files Reference

| File | Purpose |
|------|---------|
| `RobotContainer.java` | Subsystem initialization, button bindings, named commands |
| `GameStateManager.java` | Alliance color, game phases, target mode logic |
| `CalibrationManager.java` | Live tuning sliders, code generation for Constants |
| `PiShootingHelper.java` | NetworkTables bridge to Pi coprocessor |
| `AutoShootCommand.java` | Full shooting sequence with alliance window checks |
| `CommandSwerveDrivetrain.java` | Phoenix 6 swerve + PathPlanner integration |
| `pi/pi_shooting.py` | Pi coprocessor main loop (50 Hz update rate) |
| `pi/train_model.py` | ML model training from CSV data |
| `Constants.java` | All tunable values, subsystem enable flags |

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

**Debug Pi connection**:
1. Check robot is on FMS/practice field network (10.45.39.x)
2. Run `python3 test_connection.py` on Pi
3. Verify NetworkTables connection: `Pi/Status/Connected` should be true
4. Check roboRIO Driver Station log for "Pi connected" messages
5. If Pi fails, robot auto-falls back to local calibration tables
