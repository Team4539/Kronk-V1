# Kronk — FRC Team 4539 (2026 Season)

Kronk is the robot code for FRC Team 4539's 2026 competition robot.

## Robot Overview

| Component | Details |
|-----------|---------|
| **Drivetrain** | CTRE Phoenix 6 Swerve (4 × TalonFX + CANcoders, TunerX-generated) |
| **Shooter** | Single Kraken motor, fixed mount (no turret — robot rotates to aim) |
| **Intake** | Pivot + roller deploy/retract mechanism |
| **Trigger** | Ball feed to shooter |
| **Vision** | PhotonVision AprilTag camera for field-relative pose estimation |
| **LEDs** | CTRE CANdle with state-driven animations |
| **Dashboard** | Elastic Dashboard with per-phase tab layouts |

## Tech Stack

- **WPILib** 2026.2.1 (Java 17, Gradle)
- **CTRE Phoenix 6** — swerve drivetrain, motor control, CANdle LEDs
- **PathPlanner** 2026.1.2 — holonomic autonomous path following
- **PhotonVision** — AprilTag detection and robot pose estimation

## Building & Deploying

```bash
# Build
./gradlew build

# Deploy to roboRIO (team 4539)
./gradlew deploy

# Simulate (launches Driver Station GUI)
./gradlew simulateJava
```

## Controller Layout

All controls are on a single Xbox controller (port 0):

| Input | Function |
|-------|----------|
| Left Stick | Drive translation (X/Y) |
| Right Stick X | Drive rotation |
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

## Shooting System

The shooting system runs entirely on the roboRIO:

1. **PhotonVision** provides field-relative robot pose via AprilTag detection
2. **ShootingCalculator** computes distance to target, angle offset, and flywheel RPM
3. RPM is interpolated from a pose-based calibration table using inverse-distance weighting
4. Since there is no turret, the **driver rotates the entire robot** to aim

See [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md) for the shooting calibration workflow.

## Autonomous

Autonomous routines are created in PathPlanner and stored in `src/main/deploy/pathplanner/`. Paths are drawn for the Blue alliance origin; AutoBuilder automatically flips coordinates for Red alliance. Named commands are registered in `RobotContainer.java`.

## Project Structure

```
src/main/java/frc/robot/
├── Robot.java                  # Robot lifecycle (init, periodic, mode transitions)
├── RobotContainer.java         # Subsystem init, controller bindings, named commands
├── Constants.java              # All tunable values, CAN IDs, subsystem enable flags
├── GameStateManager.java       # Alliance color, game phases, target mode
├── CalibrationManager.java     # Live tuning sliders and calibration code export
├── Main.java                   # Entry point
├── commands/
│   ├── AutoShootCommand.java   # Shooting sequence with alliance window checks
│   ├── calibrations/           # Calibration-specific commands
│   └── intake/                 # Intake deploy/retract commands
├── generated/
│   └── TunerConstants.java     # Phoenix Tuner X generated swerve config (do not edit)
├── subsystems/
│   ├── CommandSwerveDrivetrain.java  # Swerve drive + PathPlanner + field visualization
│   ├── ShooterSubsystem.java        # Single-motor flywheel control
│   ├── IntakeSubsystem.java         # Pivot + roller intake
│   ├── TriggerSubsystem.java        # Ball feed mechanism
│   ├── VisionSubsystem.java         # PhotonVision camera + pose estimation
│   └── LEDSubsystem.java            # CANdle LED state machine
└── util/
    ├── ShootingCalculator.java       # Distance, angle, RPM interpolation
    ├── DashboardHelper.java          # SmartDashboard organization by category
    └── Elastic.java                  # Elastic Dashboard notifications
```

## Configuration

Subsystems can be individually enabled/disabled in `Constants.SubsystemEnabled` for bench testing without full hardware. All CAN device IDs are centralized in `Constants.CANIds`.

## AI Disclosure

AI tools (GitHub Copilot) were used in the development of this project. Specifically:

- **All code comments and documentation** (Javadoc, inline comments, this README, Calibration Guides) were written or substantially revised with AI assistance.
- **Some code** was generated or refactored with AI assistance.
- **Core robot logic, subsystem architecture, and tuning values** were designed and implemented by Team 4539 students and mentors.

All AI-generated code was reviewed and tested by the team before deployment.
