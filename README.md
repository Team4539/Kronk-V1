#  Kronk — FRC Team 4539 (2026 Season)

> **⚠️ THIS BRANCH IS NO LONGER IN ACTIVE DEVELOPMENT ⚠️**
>
> This `main` branch represents an earlier iteration of Kronk's robot code. **Active development for the new robot has moved to a separate branch:**
>
> 👉 **[`fixed-shooter` branch](https://github.com/Team4539/Kronk-V1/tree/fixed-shooter)** 👈
>
> Please refer to that branch for the latest code, bug fixes, and features. This branch is preserved for reference only.

---

## Table of Contents

- [Overview](#overview)
- [Tech Stack](#tech-stack)
- [Project Structure](#project-structure)
- [Architecture](#architecture)
  - [On-Board Shooting System](#on-board-shooting-system)
  - [Shooting-on-the-Fly (Lead Compensation)](#shooting-on-the-fly-lead-compensation)
  - [Unified Calibration System](#unified-calibration-system)
  - [Game State Management](#game-state-management)
  - [Vision Pipeline](#vision-pipeline)
  - [Command Architecture](#command-architecture)
  - [LED System](#led-system)
- [Subsystems](#subsystems)
- [Commands](#commands)
- [Constants & Configuration](#constants--configuration)
- [Building & Deploying](#building--deploying)
- [Autonomous Routines](#autonomous-routines)
- [Calibration Workflow](#calibration-workflow)
- [Controller Mappings](#controller-mappings)
- [Key Design Patterns](#key-design-patterns)
- [Critical Gotchas](#critical-gotchas)

---

## Overview

**Kronk** is the FRC robot codebase for **Team 4539** during the **2026 season**. The robot features a CTRE Phoenix 6 swerve drivetrain, a turreted shooter with dual flywheels, an articulated intake, and a full LED feedback system. All shooting intelligence — distance interpolation, turret aiming, and lead angle compensation — runs **entirely on-board the roboRIO** with no external coprocessor required.

The robot uses a **PhotonVision** camera for AprilTag-based localization, feeding pose estimates into the WPILib pose estimator fused with swerve odometry. A centralized `ShootingCalculator` computes the complete shooting solution every 20ms cycle, and all commands read from it rather than computing independently.

## Tech Stack

| Component | Technology | Version |
|-----------|-----------|---------|
| **Framework** | WPILib (GradleRIO) | 2026.2.1 |
| **Language** | Java | 17 |
| **Drivetrain** | CTRE Phoenix 6 Swerve | 26.1.1 |
| **Path Planning** | PathPlanner | 2026.1.2 |
| **Vision** | PhotonVision (PhotonLib) | — |
| **Motors** | Kraken X60 (swerve), various (turret, shooter, intake) | — |
| **LEDs** | CTRE CANdle | — |
| **Dashboard** | Elastic Dashboard + SmartDashboard | — |

---

## Project Structure

```
Kronk/
├── src/main/java/frc/robot/
│   ├── Robot.java                    # TimedRobot entry point, mode transitions
│   ├── RobotContainer.java          # Central hub: subsystem init, bindings, vision fusion
│   ├── Main.java                    # JVM entry point
│   ├── Constants.java               # All tunable values, organized by subsystem
│   ├── GameStateManager.java        # Alliance, game phases, scoring windows, target mode
│   ├── CalibrationManager.java      # Live-tuning sliders, data recording, code export
│   │
│   ├── commands/
│   │   ├── AutoShootCommand.java    # Full shooting sequence (aim + spin + feed)
│   │   ├── calibrations/
│   │   │   ├── FullShooterCalibrationCommand.java
│   │   │   ├── ShootingCalibrationCommand.java
│   │   │   ├── DistanceOffsetCalibrationCommand.java
│   │   │   ├── TurretPIDCalibrationCommand.java
│   │   │   ├── TurretRotationCalibrationCommand.java
│   │   │   ├── CalibrateTurretGearRatioCommand.java
│   │   │   └── VisionCalibrationCommand.java
│   │   ├── intake/
│   │   │   ├── DeployIntakeCommand.java
│   │   │   └── RetractIntakeCommand.java
│   │   └── turret/
│   │       └── AimTurretToPoseCommand.java  # Default turret command — tracks target
│   │
│   ├── subsystems/
│   │   ├── CommandSwerveDrivetrain.java  # Phoenix 6 swerve + PathPlanner + field viz
│   │   ├── TurretSubsystem.java         # 270° turret with bounded position control
│   │   ├── ShooterSubsystem.java        # Dual flywheel (top + bottom) RPM control
│   │   ├── TurretFeedSubsystem.java     # Ball feeder into shooter
│   │   ├── IntakeSubsystem.java         # Pivot + roller intake
│   │   ├── VisionSubsystem.java         # PhotonVision AprilTag localization
│   │   └── LEDSubsystem.java            # CANdle-driven LED animations
│   │
│   ├── util/
│   │   ├── ShootingCalculator.java   # Core shooting math (singleton)
│   │   ├── DashboardHelper.java      # Organized SmartDashboard categories
│   │   └── Elastic.java             # Elastic Dashboard notifications
│   │
│   └── generated/
│       └── TunerConstants.java       # Phoenix Tuner X generated swerve config
│
├── src/main/deploy/
│   ├── pathplanner/
│   │   ├── settings.json            # Robot dimensions, max velocities, PID
│   │   ├── autos/                   # Autonomous routines
│   │   │   ├── Training Run.auto
│   │   │   ├── Drive test.auto
│   │   │   └── calibration shot.auto
│   │   └── paths/                   # Path segments
│   │       ├── Training 1 - Close Center.path
│   │       ├── Training 2 - Left Near.path
│   │       ├── Training 3 - Far Left.path
│   │       ├── Training 4 - Right Side.path
│   │       ├── Training 5 - Close Right.path
│   │       └── ...
│   └── elastic-layout.json          # Elastic Dashboard layout
│
├── vendordeps/                      # Vendor dependency JSON files
│   ├── Phoenix6-26.1.1.json
│   ├── PathplannerLib-2026.1.2.json
│   ├── photonlib.json
│   └── WPILibNewCommands.json
│
├── build.gradle                     # GradleRIO build configuration
├── CALIBRATION_GUIDE.md             # Detailed calibration procedures
└── .github/
    └── copilot-instructions.md      # AI agent instructions for this project
```

---

## Architecture

### On-Board Shooting System

All shooting intelligence runs on the roboRIO in a single update loop — no external coprocessors, no networked calculators.

**Data flow (every 20ms):**

```
PhotonVision Camera
    ↓ AprilTag detections
VisionSubsystem (PhotonPoseEstimator)
    ↓ Robot pose estimate
RobotContainer.updateVisionPose()
    ├── Fuses vision into swerve odometry (Kalman filter)
    ├── Multi-tag: trust position + rotation
    └── Single-tag: trust position only (ignore rotation to protect gyro)
    ↓ Fused pose + chassis speeds
ShootingCalculator.update()
    ├── Resolves target position (Hub or Trench, alliance-aware)
    ├── Computes turret field position (accounts for turret offset on robot)
    ├── Calculates distance + base angle to target
    ├── Applies lead compensation for shooting-on-the-fly
    ├── Interpolates turret offset + RPMs from unified calibration table
    └── Publishes telemetry to SmartDashboard
    ↓ Cached results (turretAngle, topRPM, bottomRPM, distance, ...)
Commands read from ShootingCalculator
    ├── AutoShootCommand → aims turret, spins shooter, feeds balls
    ├── AimTurretToPoseCommand → continuously tracks target (default command)
    └── spinUpShooter (named command) → pre-spools flywheels
```

### Shooting-on-the-Fly (Lead Compensation)

`ShootingCalculator` compensates for robot motion so you can shoot while driving:

1. **Robot velocity** is converted from robot-relative to field-relative coordinates
2. **Perpendicular velocity** component relative to the target direction is extracted
3. **Flight time** is estimated: `0.5s + 0.08s × distance_meters`
4. **Lead angle** = `atan2(perpVelocity × flightTime, distance)`
5. Lead angle is **subtracted** from turret angle (aim opposite to drift direction)
6. Compensation only activates above **0.3 m/s** robot speed and **0.5m** distance

### Unified Calibration System

Rather than separate tables for RPM and turret offsets, Kronk uses a **unified calibration table** where each point stores the robot's position *relative to the target*:

```java
// Each entry: {relX, relY, bearingDeg, turretOffsetDeg, topRPM, bottomRPM}
//   relX/relY: turret position minus target position (meters)
//   bearingDeg: robot-relative angle to target (-180 to +180)
//   turretOffsetDeg: turret angle correction at this pose
//   topRPM/bottomRPM: motor speeds at this pose
```

**Why target-relative?** Using `(turretPos - targetPos)` makes the table **alliance-independent** — the same relative vector works whether you're Blue or Red, because `ShootingCalculator` resolves the correct alliance-specific target position before computing the relative offset.

**Interpolation**: Inverse-distance-weighted averaging in `(relX, relY, bearing)` space. Bearing difference is scaled by `CALIBRATION_BEARING_WEIGHT` (0.05) so 1° of bearing counts much less than 1m of distance. Points closer in this combined space have exponentially more influence (inverse-square weighting).

### Game State Management

`GameStateManager` (singleton) orchestrates match-aware behavior:

| Feature | Description |
|---------|-------------|
| **Alliance color** | Auto-detected from FMS, flips all field coordinates for Red |
| **Game phases** | `AUTO → TRANSITION → SHIFT_1-4 → END_GAME → POST_MATCH` based on match timer |
| **Scoring windows** | Alternating 25-second alliance shifts — only your alliance can score during your window |
| **Target mode** | `HUB` (scoring), `TRENCH` (shuttling), or `DISABLED` (not your turn) |
| **Auto-shuttle** | Automatically switches to trench mode when robot crosses `AUTO_SHUTTLE_BOUNDARY_X` |
| **Force shoot** | Driver override to ignore alliance window restrictions |
| **First active alliance** | Read from FMS game data message to determine shift order |

### Vision Pipeline

`VisionSubsystem` wraps PhotonVision for AprilTag-based robot localization:

- **Camera**: Arducam OV9782 USB camera, mounted at front of robot (38° pitch up)
- **Pose estimation**: `PhotonPoseEstimator` with multi-tag PNP when 2+ tags visible
- **Multi-tag fusion**: Trusted for both position and rotation (tight standard deviations)
- **Single-tag fusion**: Position trusted, rotation **ignored** (very loose theta std dev = 999.0) to prevent corrupting the gyro heading
- **Heading seeding**: On first multi-tag detection, robot pose is fully reset to bootstrap accurate heading for subsequent single-tag estimates
- **Ambiguity filtering**: Measurements with ambiguity > 0.1 are rejected

### Command Architecture

The command system follows WPILib's command-based paradigm with some key patterns:

- **`RobotContainer`**: Central wiring hub — creates subsystems, registers PathPlanner named commands, binds controllers, configures defaults
- **`AutoShootCommand`**: Full shooting orchestration. Reads from `ShootingCalculator` (never computes independently). Checks alliance windows before feeding balls. Supports incremental constructor overloads for optional subsystems (LEDs, feed motor, drivetrain)
- **`AimTurretToPoseCommand`**: Default turret command — continuously reads `ShootingCalculator.getTurretAngle()` to track the current target
- **Named commands**: Registered in `RobotContainer.registerNamedCommands()` for PathPlanner auto routines (`autoShoot`, `quickShoot`, `aimAtPose`, `spinUpShooter`, `TrainingShot`, etc.)

### LED System

`LEDSubsystem` drives a CTRE CANdle with a Y-spliced data line to two physical LED strips:

| Zone | Indices | Description |
|------|---------|-------------|
| **Onboard** | 0–7 | CANdle built-in LEDs (visible on belly pan) |
| **Shared** | 8–45 | 38 LEDs mirrored on both top/back strip AND belly strip |
| **Belly-only** | 46–65 | 20 LEDs only on belly strip (no corresponding top LEDs) |

States include alliance colors, action states (shooting, aiming, spooling, intaking, climbing), warnings for shift changes and endgame, brownout detection, and victory animations.

---

## Subsystems

| Subsystem | File | Description |
|-----------|------|-------------|
| **CommandSwerveDrivetrain** | `subsystems/CommandSwerveDrivetrain.java` | CTRE Phoenix 6 swerve drive, PathPlanner integration, SysId characterization, field visualization (Field2d with aim lines, targets, shuttle boundaries) |
| **TurretSubsystem** | `subsystems/TurretSubsystem.java` | 270° rotation turret with bounded position control (internal 75°–224°), PID control, soft limits with 10° safety margins |
| **ShooterSubsystem** | `subsystems/ShooterSubsystem.java` | Dual flywheel (top + bottom Kraken X60), velocity PID via VelocityVoltage, configurable idle RPM to keep flywheels warm |
| **TurretFeedSubsystem** | `subsystems/TurretFeedSubsystem.java` | Ball feeder motor that pushes game pieces from the turret area into the shooter wheels |
| **IntakeSubsystem** | `subsystems/IntakeSubsystem.java` | Pivot arm (0°–130° with CANcoder absolute position) + roller motor, retracted/deployed/idle positions |
| **VisionSubsystem** | `subsystems/VisionSubsystem.java` | PhotonVision camera wrapper — AprilTag detection, pose estimation, multi/single-tag handling, distance/angle to targets |
| **LEDSubsystem** | `subsystems/LEDSubsystem.java` | CANdle LED controller — alliance colors, action animations, warnings, victory patterns |

All subsystems are **conditionally initialized** in `RobotContainer` based on `Constants.SubsystemEnabled` flags. Set any flag to `false` to disable that subsystem for bench testing without hardware.

---

## Commands

### Core Commands

| Command | Trigger | Description |
|---------|---------|-------------|
| **AutoShootCommand** | Driver A / Operator RT | Full auto-shoot: aim turret → spin up shooter → check alliance window → feed ball |
| **AimTurretToPoseCommand** | Default (always running) | Continuously tracks current target via ShootingCalculator |
| **DeployIntakeCommand** | Driver RB / Operator A | Extends intake pivot, runs rollers |
| **RetractIntakeCommand** | Release Driver RB / Operator A | Retracts intake pivot, stops rollers |

### Calibration Commands

| Command | Purpose |
|---------|---------|
| **FullShooterCalibrationCommand** | Combined turret + shooter calibration — record position/RPM/offset points |
| **ShootingCalibrationCommand** | RPM-only calibration at various distances |
| **TurretRotationCalibrationCommand** | Turret angle offset calibration by bearing |
| **TurretPIDCalibrationCommand** | Live PID tuning for turret |
| **CalibrateTurretGearRatioCommand** | Determine exact turret gear ratio |
| **DistanceOffsetCalibrationCommand** | Fine-tune distance measurement offsets |
| **VisionCalibrationCommand** | Vision pipeline tuning and verification |

### PathPlanner Named Commands

| Name | Description |
|------|-------------|
| `autoShoot` | AutoShootCommand with 3.0s timeout |
| `quickShoot` | AutoShootCommand with 1.5s timeout |
| `aimAtPose` | AimTurretToPoseCommand with 2.0s timeout |
| `spinUpShooter` | Pre-spool flywheels to calculated RPMs |
| `stopShooter` | Stop shooter motors |
| `centerTurret` | Return turret to 0° (forward) |
| `TrainingShot` | Shoot without alliance window checks (for practice) |
| `enableShuttleMode` / `disableShuttleMode` | Toggle shuttle (trench) targeting |
| `wait0.5` / `wait1.0` / `wait2.0` | Timed delays |

---

## Constants & Configuration

All tunable values live in `Constants.java`, organized by nested static class:

| Class | Purpose |
|-------|---------|
| `Constants.Robot` | Physical dimensions (27" × 28") |
| `Constants.CANIds` | All CAN bus device IDs (swerve IDs are in TunerConstants) |
| `Constants.SubsystemEnabled` | Toggle subsystems on/off for bench testing |
| `Constants.Field` | Field geometry, hub/trench positions (Blue + Red variants), shuttle boundary |
| `Constants.Shooter` | Motor config, ball physics, unified calibration table (`SHOOTING_CALIBRATION`), idle RPM |
| `Constants.TurretFeed` | Feed motor speeds (idle, shoot, stop) |
| `Constants.Intake` | Pivot angles, PID, roller speeds, CANcoder offset |
| `Constants.Vision` | Camera name, mounting offsets, standard deviations, ambiguity threshold |
| `Constants.Driver` | Controller ports, deadbands, max speeds, slow-mode multiplier |
| `Constants.Turret` | Gear ratio, PID, angle limits (internal 75°–224°), position offsets, per-tag offsets |
| `Constants.Tags` | AprilTag ID arrays for Blue/Red Hub and Trench |
| `Constants.LEDs` | LED counts, zones, colors, animation speeds |

---

## Building & Deploying

### Prerequisites

- **WPILib 2026.2.1** (includes JDK 17, Gradle, VS Code extensions)
- **Phoenix Tuner X** for swerve configuration (do NOT hand-edit `generated/TunerConstants.java`)
- **PhotonVision** installed on camera (http://photonvision.local:5800)

### Commands

```bash
# Build robot code
./gradlew build

# Deploy to roboRIO (team number from .wpilib/wpilib_preferences.json)
./gradlew deploy

# Simulate robot code (opens Driver Station + GUI)
./gradlew simulateJava
```

---

## Autonomous Routines

Autonomous paths are created in the **PathPlanner GUI** and stored in `src/main/deploy/pathplanner/`.

### Available Autos

| Auto | Description |
|------|-------------|
| **Training Run** | Multi-point path hitting 5 shooting positions with `TrainingShot` at each |
| **Drive test** | Simple drive test for verifying swerve functionality |
| **calibration shot** | Static shot for verifying shooting at a known position |

### PathPlanner Configuration

From `settings.json`:
- **Max velocity**: 3.0 m/s
- **Max acceleration**: 3.0 m/s²
- **Max angular velocity**: 540°/s
- **Max angular acceleration**: 720°/s²
- **Robot mass**: 74.088 kg
- **Drive motors**: Kraken X60, 5.143:1 gearing, 0.048m wheel radius
- **Module positions**: ±0.273m from center (square layout)

**Alliance flipping** is handled automatically by PathPlanner's `AutoBuilder` — paths are drawn for Blue alliance origin, and the system flips them for Red at runtime.

---

## Calibration Workflow

See [`CALIBRATION_GUIDE.md`](CALIBRATION_GUIDE.md) for detailed step-by-step procedures.

### Quick Overview

1. **Start calibration session**: Press "Start Calibration" on SmartDashboard
   - Zeros ALL live offset sliders
   - Bypasses baked-in `Constants` offsets for a raw baseline
2. **Drive to a position** near the target
3. **Adjust sliders** on SmartDashboard:
   - `Tuning/Turret/AngleOffset` — turret angle correction
   - `Tuning/Turret/RotationOffset` — bearing-dependent correction
   - `Tuning/Shooter/TopRPM` / `BottomRPM` — flywheel speeds
4. **Feed a test shot** (Operator POV Down, or "Feed Shot" dashboard button)
5. **Record the point** — press "RecordPoint" to save `{relX, relY, bearing, turretOffset, topRPM, bottomRPM}`
6. **Repeat** at multiple positions and orientations
7. **Export** — press "PrintTable" to get copy-paste Java code for `Constants.Shooter.SHOOTING_CALIBRATION`
8. **End session** — press "End Calibration" to re-enable baked-in offsets

---

## Controller Mappings

### Driver Controller (Port 0)

| Button | Action |
|--------|--------|
| **Left Stick** | Swerve translation (field-centric by default) |
| **Right Stick X** | Swerve rotation |
| **RT (axis)** | Slow mode (30% speed) |
| **LT** | X-lock brake (wheels in X pattern) |
| **RB (hold)** | Deploy intake; release to retract |
| **LB** | Toggle field-centric / robot-centric |
| **A (hold)** | Auto-shoot |
| **B** | Toggle force shoot (ignore alliance windows) |
| **Y** | Reset gyro heading |
| **Start** | Reset gyro heading (alternate) |
| **POV Up** | Point wheels forward |

### Operator Controller (Port 1)

| Button | Action |
|--------|--------|
| **RT** | Auto-shoot |
| **LT** | Pre-spool shooter to calculated RPMs |
| **RB** | Toggle shuttle mode (Hub ↔ Trench) |
| **LB** | Toggle force shoot |
| **A (hold)** | Deploy intake; release to retract |
| **B (hold)** | Aim turret only (no shooting) |
| **X** | E-stop all motors (shooter + feed) |
| **Y** | Center turret (0°) |
| **Start** | Reset game state |
| **Back** | Clear shuttle mode override (re-enable auto-shuttle) |
| **POV Down** | Feed ball into shooter (for calibration) |

---

## Key Design Patterns

### Subsystem Nullability

All subsystems in `RobotContainer` are nullable and guarded with null checks:

```java
private final TurretSubsystem turret;  // Can be null if disabled
// ...
if (turret != null) {
    turret.setTargetAngle(angle);
}
```

This allows testing individual subsystems without requiring the full robot hardware to be connected.

### Singleton Services

`ShootingCalculator`, `GameStateManager`, and `CalibrationManager` are singletons accessed via `getInstance()`. They hold shared state that multiple commands and subsystems need:

- **ShootingCalculator**: Updated once per cycle in `RobotContainer.updateVisionPose()`, read by any command that needs shooting data
- **GameStateManager**: Updated once per cycle, provides alliance/phase/target info to all consumers
- **CalibrationManager**: Updated once per cycle, reads SmartDashboard sliders and feeds offsets into ShootingCalculator

### Commands Never Calculate Independently

`AutoShootCommand` and `AimTurretToPoseCommand` **read** from `ShootingCalculator` — they never duplicate the distance/angle/RPM calculations. This ensures a single source of truth and makes calibration adjustments apply everywhere instantly.

### Alliance-Independent Calibration

The unified calibration table stores positions *relative to the target* (`turretPos - targetPos`). Since `ShootingCalculator` resolves the correct target position for the current alliance before computing relative offsets, the same calibration data works for both Blue and Red alliance without any mirroring.

---

## Critical Gotchas

1. **Alliance flipping**: All field coordinates in `Constants.Field` have Blue and Red variants. `ShootingCalculator` and `GameStateManager` auto-resolve based on `DriverStation.getAlliance()`.

2. **Turret angle limits**: 270° rotation range with internal bounds of 75°–224°. There's a 10° safety margin. **Never** command angles outside this range or the turret will fault. The dead zone spans ~211° of internal angle (224° → through 360°/0° → 75°).

3. **Phoenix 6 swerve**: `generated/TunerConstants.java` is **generated by Phoenix Tuner X**. Do not hand-edit — regenerate using the Tuner X application.

4. **Single-tag rotation is unreliable**: Vision rotation from single-tag PNP is intentionally discarded (theta std dev = 999.0). Only multi-tag PNP provides trusted rotation. If the robot only sees single tags, the gyro heading from odometry is preserved.

5. **PathPlanner coordinates**: All paths are drawn for **Blue alliance origin**. AutoBuilder flips for Red automatically. Don't manually mirror paths.

6. **Voltage compensation**: Shooter RPMs use CTRE VelocityVoltage closed-loop control on the motor controllers, not simple duty cycle.

7. **Calibration mode vs normal mode**: When `calibrationMode = true` in `ShootingCalculator`, ALL baked-in offsets from `Constants` are bypassed. Only live slider values apply. This prevents stacking new calibration on top of old calibration data.

8. **Vision heading seeding**: The robot doesn't trust its heading until it gets a multi-tag vision measurement. Until then, single-tag X/Y is used but rotation comes from the gyro (which starts at 0°). Manually seed heading with the Y button if needed.

---

## AI Disclosure

Portions of this codebase were written with the assistance of **GitHub Copilot** (AI). This includes:

- **Code comments and documentation** — Javadoc headers, inline explanations, and architecture descriptions were drafted or refined with AI assistance
- **This README** — generated with AI assistance based on the actual codebase contents
- **Boilerplate and utility code** — some repetitive patterns, helper methods, and configuration scaffolding were AI-assisted
- **Calibration guide** — documentation in `CALIBRATION_GUIDE.md` was written with AI assistance

All AI-generated and AI-assisted code was **reviewed, tested, and validated** by team members before being committed. The core algorithms (shooting math, lead compensation, interpolation, game state logic) reflect the team's engineering design and were tuned through real-world testing on the robot.

---

## License

This project uses WPILib and is subject to the [WPILib BSD License](WPILib-License.md).

---

*Built with ❤️ by FRC Team 4539*
