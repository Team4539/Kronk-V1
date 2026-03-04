package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.calibrations.DistanceOffsetCalibrationCommand;
import frc.robot.commands.calibrations.FullShooterCalibrationCommand;
import frc.robot.commands.calibrations.VisionCalibrationCommand;
import frc.robot.commands.calibrations.ShootingCalibrationCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.IntakeJiggleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.ShootingCalculator;

/**
 * Central hub: subsystems, commands, and controller bindings.
 * Enable/disable subsystems via Constants.SubsystemEnabled.
 */
public class RobotContainer {

    // Singletons
    private final GameStateManager gameState = GameStateManager.getInstance();
    private final CalibrationManager calibration = CalibrationManager.getInstance();
    private final ShootingCalculator shootingCalc = ShootingCalculator.getInstance();

    // Subsystems (null when disabled in Constants.SubsystemEnabled)
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final TriggerSubsystem trigger;
    private final IntakeSubsystem intake;
    private final LEDSubsystem leds;

    // Controllers - single driver controller
    private final CommandXboxController driver =
            new CommandXboxController(Constants.Driver.DRIVER_CONTROLLER_PORT);

    // Swerve requests
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Driver.MAX_DRIVE_SPEED_MPS * Constants.Driver.STICK_DEADBAND)
            .withRotationalDeadband(Constants.Driver.MAX_ANGULAR_SPEED_RAD * Constants.Driver.STICK_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Separate instance for auto-aim so it doesn't share mutable state with the default drive request
    private final SwerveRequest.FieldCentric aimFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Driver.MAX_DRIVE_SPEED_MPS * Constants.Driver.STICK_DEADBAND)
            .withRotationalDeadband(0) // No rotational deadband during auto-aim
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(Constants.Driver.MAX_DRIVE_SPEED_MPS * Constants.Driver.STICK_DEADBAND)
            .withRotationalDeadband(Constants.Driver.MAX_ANGULAR_SPEED_RAD * Constants.Driver.STICK_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt pointWheels = new SwerveRequest.PointWheelsAt();

    // State
    private boolean useRobotCentric = false;
    private final SendableChooser<Command> autoChooser;

    /** Whether vision has seeded the gyro heading at least once since boot. */
    private boolean hasSeededHeadingFromVision = false;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================

    public RobotContainer() {
        // 1. Init subsystems
        drivetrain = Constants.SubsystemEnabled.DRIVETRAIN  ? TunerConstants.createDrivetrain() : null;
        shooter    = Constants.SubsystemEnabled.SHOOTER     ? new ShooterSubsystem()             : null;
        vision     = Constants.SubsystemEnabled.VISION      ? new VisionSubsystem()              : null;
        trigger    = Constants.SubsystemEnabled.TRIGGER     ? new TriggerSubsystem()             : null;
        intake     = Constants.SubsystemEnabled.INTAKE      ? new IntakeSubsystem()              : null;
        leds       = Constants.SubsystemEnabled.LEDS        ? new LEDSubsystem()                 : null;

        logSubsystemStatus();

        // 2. Register PathPlanner named commands (must happen before autoChooser)
        registerNamedCommands();

        // 3. Auto chooser
        autoChooser = AutoBuilder.isConfigured()
                ? AutoBuilder.buildAutoChooser()
                : new SendableChooser<>();

        // 4. Bindings and defaults
        configureBindings();
        configureDefaultCommands();
        configureDashboard();
    }

    private void logSubsystemStatus() {
        System.out.println("[Subsystems] Drive=" + (drivetrain != null)
                + " Shooter=" + (shooter != null)
                + " Vision=" + (vision != null) + " Trigger=" + (trigger != null)
                + " Intake=" + (intake != null) + " LEDs=" + (leds != null));
    }

    // =========================================================================
    // PATHPLANNER NAMED COMMANDS
    // =========================================================================

    private void registerNamedCommands() {
        if (shooter != null && vision != null) {
            NamedCommands.registerCommand("autoShoot",
                    new AutoShootCommand(shooter, vision, leds, trigger, drivetrain)
                            .withTimeout(3.0));
            NamedCommands.registerCommand("quickShoot",
                    new AutoShootCommand(shooter, vision, leds, trigger, drivetrain)
                            .withTimeout(1.5));
            NamedCommands.registerCommand("TrainingShot",
                    new AutoShootCommand(shooter, vision, leds, trigger, drivetrain)
                            .withTimeout(2.5));
        }
        if (shooter != null) {
            NamedCommands.registerCommand("spinUpShooter",
                    Commands.run(() -> shooter.setTargetRPM(
                            shootingCalc.getTargetRPM()), shooter));
            NamedCommands.registerCommand("stopShooter",
                    Commands.runOnce(() -> shooter.stop(), shooter));
        }
        NamedCommands.registerCommand("enableShuttleMode",
                Commands.runOnce(() -> gameState.setShuttleMode(true)));
        NamedCommands.registerCommand("disableShuttleMode",
                Commands.runOnce(() -> gameState.setShuttleMode(false)));

        // Auto-aim: rotate robot to face the current target (hub or trench).
        // Uses robot-relative angleToTarget with simple P controller for rotation rate.
        if (drivetrain != null) {
            NamedCommands.registerCommand("aimAtPose",
                    drivetrain.applyRequest(() -> {
                        double errorDeg = shootingCalc.getAngleToTarget();
                        double omega = -Math.toRadians(errorDeg) * Constants.Driver.AIM_HEADING_KP;
                        return aimFieldCentric
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(omega);
                    }).until(() -> Math.abs(shootingCalc.getAngleToTarget())
                                    < Constants.Driver.AIM_TOLERANCE_DEG)
                     .withTimeout(1.5));
        }

        NamedCommands.registerCommand("wait0.5", Commands.waitSeconds(0.5));
        NamedCommands.registerCommand("wait1.0", Commands.waitSeconds(1.0));
        NamedCommands.registerCommand("wait2.0", Commands.waitSeconds(2.0));
    }

    // =========================================================================
    // CONTROLLER BINDINGS
    // =========================================================================

    private void configureBindings() {
        configureSingleControllerBindings();
    }

    /**
     * All controls on a single Xbox controller:
     * 
     * LEFT STICK:  Drive X/Y (translation)
     * RIGHT STICK: Rotation
     * RT:          Slow mode (proportional)
     * LT:          Pre-spool shooter + auto-aim (hold)
     * RB:          Intake deploy (hold) / retract (release)
     * LB:          Auto-shoot + auto-aim (hold to shoot)
     * A:           E-stop all motors
     * B:           Force shoot toggle
     * X:           Shuttle mode toggle
     * Y:           Reset gyro
     * START:       Reset game state
     * BACK:        Toggle field/robot centric
     * POV UP:      Point wheels forward (hold)
     * POV DOWN:    Feed shot (calibration, hold)
     */
    private void configureSingleControllerBindings() {
        // LT: Pre-spool shooter + auto-aim (hold)
        // Spools shooter to calculated RPM AND rotates robot toward target via camera.
        // Driver keeps translation control; rotation auto-aims toward target.
        if (shooter != null) {
            Command spoolCmd = Commands.run(() -> {
                shooter.setTargetRPM(shootingCalc.getTargetRPM());
                if (leds != null) leds.setAction(LEDSubsystem.ActionState.SPOOLING);
            }, shooter).finallyDo(() -> {
                shooter.stop();
                if (leds != null) leds.clearAction();
            });

            if (drivetrain != null) {
                // Auto-aim: rotate toward target while driver controls translation
                Command aimCmd = drivetrain.applyRequest(() -> {
                    double slow = driver.getRightTriggerAxis() > 0.3
                            ? Constants.Driver.SLOW_MODE_MULTIPLIER : 1.0;
                    double vx = -driver.getLeftY() * Constants.Driver.MAX_DRIVE_SPEED_MPS * slow;
                    double vy = -driver.getLeftX() * Constants.Driver.MAX_DRIVE_SPEED_MPS * slow;
                    double errorDeg = shootingCalc.getAngleToTarget();
                    double omega = -Math.toRadians(errorDeg) * Constants.Driver.AIM_HEADING_KP;
                    return aimFieldCentric
                            .withVelocityX(vx)
                            .withVelocityY(vy)
                            .withRotationalRate(omega);
                });
                driver.leftTrigger(0.5).whileTrue(spoolCmd.alongWith(aimCmd));
            } else {
                driver.leftTrigger(0.5).whileTrue(spoolCmd);
            }
        }

        // LB: Auto-shoot (hold) — also jiggles intake to keep balls feeding
        // Auto-aims robot toward target via camera while shooting.
        if (shooter != null && vision != null) {
            Command shootCmd = new AutoShootCommand(shooter, vision, leds, trigger, drivetrain);
            
            Command fullCmd = shootCmd;
            if (intake != null) {
                fullCmd = fullCmd.alongWith(new IntakeJiggleCommand(intake));
            }
            // Auto-aim: rotate toward target while driver controls translation
            if (drivetrain != null) {
                Command aimCmd = drivetrain.applyRequest(() -> {
                    double slow = driver.getRightTriggerAxis() > 0.3
                            ? Constants.Driver.SLOW_MODE_MULTIPLIER : 1.0;
                    double vx = -driver.getLeftY() * Constants.Driver.MAX_DRIVE_SPEED_MPS * slow;
                    double vy = -driver.getLeftX() * Constants.Driver.MAX_DRIVE_SPEED_MPS * slow;
                    double errorDeg = shootingCalc.getAngleToTarget();
                    double omega = -Math.toRadians(errorDeg) * Constants.Driver.AIM_HEADING_KP;
                    return aimFieldCentric
                            .withVelocityX(vx)
                            .withVelocityY(vy)
                            .withRotationalRate(omega);
                });
                fullCmd = fullCmd.alongWith(aimCmd);
            }
            driver.leftBumper().whileTrue(fullCmd);
        }

        // RB: Intake deploy/retract (hold/release)
        // DeployIntakeCommand runs continuously with stall detection while held.
        // On release, whileTrue interrupts it (calls end → stops rollers),
        // then finallyDo retracts the pivot and clears LEDs.
        if (intake != null) {
            driver.rightBumper().whileTrue(
                    new DeployIntakeCommand(intake)
                            .alongWith(ledAction(LEDSubsystem.ActionState.INTAKING))
                            .finallyDo(() -> {
                                intake.stopAndRetract();
                                if (leds != null) leds.clearAction();
                            }));
        }

        // A: E-stop all motors
        driver.a().onTrue(Commands.runOnce(() -> {
            if (shooter != null) shooter.stop();
            if (trigger != null) trigger.stop();
            notify("E-STOP");
        }));

        // B: Force shoot toggle
        driver.b().onTrue(Commands.runOnce(() -> {
            boolean on = !gameState.isForceShootEnabled();
            gameState.setForceShootEnabled(on);
            notify(on ? "Force Shoot ON" : "Force Shoot OFF");
        }));

        // X: Shuttle mode toggle
        driver.x().onTrue(Commands.runOnce(() -> {
            boolean on = !gameState.isShuttleMode();
            gameState.setShuttleMode(on);
            notify(on ? "Shuttle Mode ON" : "Hub Mode");
        }));

        // Y: Reset gyro
        if (drivetrain != null) {
            driver.y().onTrue(Commands.runOnce(() -> {
                drivetrain.seedFieldCentric();
                notify("Gyro Reset");
            }));
        }

        // Start: Reset game state
        driver.start().onTrue(Commands.runOnce(() -> {
            gameState.reset();
            notify("Game State Reset");
        }));

        // Back: Toggle field/robot centric
        driver.back().onTrue(Commands.runOnce(() -> {
            useRobotCentric = !useRobotCentric;
            notify(useRobotCentric ? "Robot Centric" : "Field Centric");
        }));

        // POV Up: Point wheels forward
        if (drivetrain != null) {
            driver.povUp().whileTrue(
                    drivetrain.applyRequest(() -> pointWheels.withModuleDirection(Rotation2d.kZero)));
        }

        // POV Down: Feed shot (calibration - feeds ball into shooter while held)
        if (trigger != null) {
            driver.povDown().whileTrue(
                    Commands.startEnd(
                            () -> trigger.setShoot(),
                            () -> trigger.stop(),
                            trigger));
        }
    }

    // =========================================================================
    // DEFAULT COMMANDS
    // =========================================================================

    private void configureDefaultCommands() {
        // Drivetrain: swerve drive with slow-mode on RT
        if (drivetrain != null) {
            drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
                // SAFETY: Do not drive while disabled; send idle/brake request
                if (DriverStation.isDisabled()) {
                    return brake;
                }

                double slow = driver.getRightTriggerAxis() > 0.3
                        ? Constants.Driver.SLOW_MODE_MULTIPLIER : 1.0;
                double vx = -driver.getLeftY()  * Constants.Driver.MAX_DRIVE_SPEED_MPS  * slow;
                double vy = -driver.getLeftX()  * Constants.Driver.MAX_DRIVE_SPEED_MPS  * slow;
                double vr = driver.getRightX() * Constants.Driver.MAX_ANGULAR_SPEED_RAD * slow;

                if (useRobotCentric) {
                    return robotCentric.withVelocityX(vx).withVelocityY(vy).withRotationalRate(vr);
                }
                return fieldCentric.withVelocityX(vx).withVelocityY(vy).withRotationalRate(vr);
            }));
        }

        // Shooter: idle at low RPM to keep flywheels warm.
        // Tunable via SmartDashboard "Shooter/IdleRPM". Set to 0 to fully stop.
        // Full spool-up only happens when driver holds LB or during AutoShoot.
        if (shooter != null) {
            SmartDashboard.putNumber("Shooter/IdleRPM", Constants.Shooter.DEFAULT_IDLE_RPM);
            shooter.setDefaultCommand(Commands.run(() -> {
                double idleRPM = SmartDashboard.getNumber("Shooter/IdleRPM",
                        Constants.Shooter.DEFAULT_IDLE_RPM);
                if (idleRPM > 0.1) {
                    shooter.setTargetRPM(idleRPM);
                } else {
                    shooter.stop();
                }
            }, shooter));
        }
    }

    // =========================================================================
    // DASHBOARD (auto chooser + calibration commands only)
    // =========================================================================

    private void configureDashboard() {
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Calibration commands for pit use
        if (shooter != null && vision != null) {
            SmartDashboard.putData("Tuning/Cal: Full Shooter",
                    new FullShooterCalibrationCommand(shooter, vision));
            SmartDashboard.putData("Tuning/Cal: Shooting",
                    new ShootingCalibrationCommand(shooter, vision));
            SmartDashboard.putData("Tuning/Cal: Distance Offset",
                    new DistanceOffsetCalibrationCommand(shooter, vision));
        }
        if (vision != null) {
            SmartDashboard.putData("Tuning/Cal: Vision",
                    new VisionCalibrationCommand(vision));
        }

        // Quick-fire dashboard button for calibration: feeds a ball into the shooter
        // without requiring alliance windows or the full AutoShoot sequence.
        if (trigger != null) {
            SmartDashboard.putData("Tuning/Feed Shot",
                    Commands.startEnd(
                            () -> trigger.setShoot(),
                            () -> trigger.stop(),
                            trigger
                    ).withName("Feed Shot"));
        }
    }

    // =========================================================================
    // PERIODIC - called from Robot.robotPeriodic()
    // =========================================================================

    public void updateVisionPose() {
        calibration.update();

        // Update shooting calculator with current pose
        if (drivetrain != null) {
            // In calibration mode, pass the absolute RPM slider value so
            // ShootingCalculator uses it directly (bypasses interpolation table).
            // In normal mode, pass the RPM offset added on top of interpolated RPM.
            double rpmParam = shootingCalc.isCalibrationMode()
                    ? calibration.getShooterRPM()
                    : calibration.getRPMOffset();
            shootingCalc.update(
                    drivetrain.getState().Pose,
                    drivetrain.getState().Speeds,
                    gameState.getTargetMode(),
                    rpmParam);
            calibration.setCurrentDistance(shootingCalc.getDistance());
        }

        if (vision == null || drivetrain == null) return;

        // Feed drivetrain pose to vision for fallback
        vision.setDrivetrainPose(drivetrain.getState().Pose);

        // Feed gyro heading to vision pose estimator for better single-tag estimates.
        double yawDegrees = drivetrain.getState().Pose.getRotation().getDegrees();
        double yawRate = Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond);
        vision.setRobotOrientation(yawDegrees, yawRate, 0, 0, 0, 0);

        // Auto-shuttle zone check
        gameState.update(isInShuttleZone());

        // --- Heading seeding (first time we get any vision pose) ---
        if (!hasSeededHeadingFromVision && vision.hasVisionPose()) {
            edu.wpi.first.math.geometry.Pose2d seedPose = vision.getRobotPose();
            if (vision.isMultiTag()) {
                // Multi-tag gives accurate rotation — hard-reset the whole pose
                drivetrain.resetPose(seedPose);
                hasSeededHeadingFromVision = true;
                vision.notifyHeadingSeeded();
                System.out.println("[RobotContainer] Vision pose initialized (MultiTag): "
                        + String.format("(%.2f, %.2f) @ %.1f\u00b0",
                                seedPose.getX(), seedPose.getY(),
                                seedPose.getRotation().getDegrees()));
                Elastic.sendNotification(
                        new Elastic.Notification(NotificationLevel.INFO,
                                "Vision Pose Initialized",
                                String.format("MultiTag: (%.1f, %.1f) @ %.1f\u00b0 from %d tags",
                                        seedPose.getX(), seedPose.getY(),
                                        seedPose.getRotation().getDegrees(),
                                        vision.getTargetCount())));
            } else {
                // Single-tag: trust X/Y, keep gyro heading
                hasSeededHeadingFromVision = true;
                vision.notifyHeadingSeeded();
                System.out.println("[RobotContainer] Vision X/Y initialized (SingleTag, gyro heading kept): "
                        + String.format("(%.2f, %.2f)", seedPose.getX(), seedPose.getY()));
                Elastic.sendNotification(
                        new Elastic.Notification(NotificationLevel.INFO,
                                "Vision X/Y Initialized",
                                String.format("SingleTag ID %d: (%.1f, %.1f) - heading from gyro",
                                        vision.getTargetId(),
                                        seedPose.getX(), seedPose.getY())));
            }
        }

        // --- Fuse EACH camera's pose independently into drivetrain odometry ---
        // This gives the Kalman filter two measurements per cycle when both cameras see tags.
        if (vision.hasFrontPose()) {
            fuseCamera(vision.getFrontPose(), vision.getFrontTimestamp(),
                       vision.isFrontMultiTag(), vision.getFrontTargetCount());
        }
        if (vision.hasRearPose()) {
            fuseCamera(vision.getRearPose(), vision.getRearTimestamp(),
                       vision.isRearMultiTag(), vision.getRearTargetCount());
        }
    }

    private boolean isInShuttleZone() {
        if (drivetrain == null) return false;
        double x = drivetrain.getState().Pose.getX();
        double boundary = Constants.Field.AUTO_SHUTTLE_BOUNDARY_X;
        boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Blue;
        return isBlue ? x > boundary : x < (Constants.Field.FIELD_LENGTH_METERS - boundary);
    }

    // =========================================================================
    // PUBLIC GETTERS (used by Robot.java)
    // =========================================================================

    public Command getAutonomousCommand() { return autoChooser.getSelected(); }
    public LEDSubsystem getLEDs() { return leds; }

    /** Stop all motors - called from Robot.disabledInit(). */
    public void stopAllMotors() {
        if (drivetrain != null) drivetrain.setControl(brake);
        if (shooter != null)    shooter.stop();
        if (trigger != null)    trigger.stop();
        if (intake != null) {
            intake.stopRollers();
            intake.retract();
        }
    }

    // =========================================================================
    // HELPERS
    // =========================================================================

    /**
     * Fuse a single camera's pose into the drivetrain Kalman filter.
     * Both multi-tag and single-tag results now feed rotation to support auto-aim.
     * Single-tag uses a looser theta std dev via the scale factor in VisionSubsystem.
     */
    private void fuseCamera(edu.wpi.first.math.geometry.Pose2d pose, double timestamp,
                            boolean isMultiTag, int tagCount) {
        double[] std = vision.getStdDevsForCamera(isMultiTag, tagCount);
        drivetrain.addVisionMeasurement(pose, timestamp, VecBuilder.fill(std[0], std[1], std[2]));
    }

    private Command ledAction(LEDSubsystem.ActionState action) {
        return Commands.runOnce(() -> { if (leds != null) leds.setAction(action); });
    }

    private Command ledClear() {
        return Commands.runOnce(() -> { if (leds != null) leds.clearAction(); });
    }

    private void notify(String title) {
        Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle(title)
                .withDisplaySeconds(1.5));
    }
}
