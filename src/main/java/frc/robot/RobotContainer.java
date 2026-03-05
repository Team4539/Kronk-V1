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
    /** Persistent E-stop flag. When true, all motor outputs are suppressed until cleared. */
    private boolean eStopActive = false;
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
                        double omega = aimOmega(errorDeg);
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
     * RIGHT STICK: Rotation (manual, overridden by auto-aim when LT held)
     * RT:          Slow mode (proportional)
     * LT:          Auto-aim + pre-spool shooter + deploy intake (hold)
     *              - Default drive command auto-rotates toward target when LT > 0.5
     *              - Simultaneously spools shooter to calculated RPM
     *              - Deploys intake so ball is ready to feed on LB press
     * RB:          Intake deploy (hold) / retract (release)
     * LB:          Auto-shoot + auto-aim + jiggle (hold to shoot)
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
        // LT: Pre-spool shooter + deploy intake (hold)
        // Spools shooter to calculated RPM and drops the intake so the ball
        // is ready to feed immediately when LB (shoot) is pressed.
        // Auto-aim rotation is handled by the default drive command
        // (detects LT > 0.5 and auto-rotates toward target).
        if (shooter != null) {
            Command spoolCmd = Commands.run(() -> {
                shooter.setTargetRPM(shootingCalc.getTargetRPM());
                if (leds != null) leds.setAction(LEDSubsystem.ActionState.SPOOLING);
            }, shooter).finallyDo(() -> {
                shooter.stop();
                if (leds != null) leds.clearAction();
            });

            // Deploy intake alongside spool so it's ready to feed
            if (intake != null) {
                Command intakeDeployCmd = Commands.startEnd(
                        () -> intake.deploy(),
                        () -> intake.stopAndRetract(),
                        intake);
                driver.leftTrigger(0.5).whileTrue(spoolCmd.alongWith(intakeDeployCmd));
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
                    double omega = aimOmega(shootingCalc.getAngleToTarget());
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

        // A: E-stop all motors (persistent until mode change)
        driver.a().onTrue(Commands.runOnce(() -> {
            eStopActive = true;
            stopAllMotors();
            if (leds != null) leds.setAction(LEDSubsystem.ActionState.ESTOP);
            notify("E-STOP ACTIVE");
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

        // POV Down: Feed shot (calibration - mimics the exact normal shooting sequence)
        // Runs trigger + intake jiggle just like LB auto-shoot does, so calibration
        // shots behave identically to real match shots.
        if (trigger != null) {
            Command feedCmd = Commands.startEnd(
                    () -> trigger.setShoot(),
                    () -> trigger.stop(),
                    trigger);
            if (intake != null) {
                feedCmd = feedCmd.alongWith(new IntakeJiggleCommand(intake));
            }
            driver.povDown().whileTrue(feedCmd);
        }
    }

    // =========================================================================
    // DEFAULT COMMANDS
    // =========================================================================

    private void configureDefaultCommands() {
        // Drivetrain: swerve drive with auto-aim integration.
        // When LT is held past 0.5, rotation automatically tracks the shooting target
        // (P controller on angleToTarget). Otherwise, right stick controls rotation normally.
        // Translation (left stick) and slow mode (RT) always work regardless of auto-aim.
        if (drivetrain != null) {
            drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
                // SAFETY: Do not drive while disabled or E-stopped; send idle/brake request
                if (DriverStation.isDisabled() || eStopActive) {
                    return brake;
                }

                double slow = driver.getRightTriggerAxis() > 0.3
                        ? Constants.Driver.SLOW_MODE_MULTIPLIER : 1.0;
                double vx = -driver.getLeftY()  * Constants.Driver.MAX_DRIVE_SPEED_MPS  * slow;
                double vy = -driver.getLeftX()  * Constants.Driver.MAX_DRIVE_SPEED_MPS  * slow;

                // On-the-fly auto-aim: LT held → auto-rotate toward target
                boolean autoAim = driver.getLeftTriggerAxis() > 0.5;
                if (autoAim) {
                    double omega = aimOmega(shootingCalc.getAngleToTarget());
                    // Use aimFieldCentric (no rotational deadband) for precise auto-aim
                    return aimFieldCentric.withVelocityX(vx).withVelocityY(vy)
                            .withRotationalRate(omega);
                }

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
                // SAFETY: Do not spin shooter while disabled or E-stopped
                if (DriverStation.isDisabled() || eStopActive) {
                    shooter.stop();
                    return;
                }
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
        // mimicking the exact normal shooting sequence (trigger + intake jiggle).
        if (trigger != null) {
            Command dashFeedCmd = Commands.startEnd(
                            () -> trigger.setShoot(),
                            () -> trigger.stop(),
                            trigger
                    ).withName("Feed Shot");
            if (intake != null) {
                dashFeedCmd = dashFeedCmd.alongWith(
                        new IntakeJiggleCommand(intake)).withName("Feed Shot");
            }
            SmartDashboard.putData("Tuning/Feed Shot", dashFeedCmd);
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
    /** Returns true when the driver E-stop is active. */
    public boolean isEStopActive() { return eStopActive; }

    /** Clear the E-stop flag. Called on mode transitions (auto/teleop init). */
    public void clearEStop() {
        if (eStopActive) {
            eStopActive = false;
            notify("E-Stop Cleared");
        }
    }

    /** Stop all motors - called from Robot.disabledInit() and E-stop. */
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

    /** Previous aim error in radians — used for D term to damp oscillation */
    private double previousAimErrorRad = 0.0;

    /**
     * Computes the auto-aim rotational rate (rad/s) from a heading error (degrees).
     * PD controller with slow-approach zone, deadband, and saturation:
     *   - P term drives toward the target
     *   - Inside AIM_SLOW_ZONE_DEG, effective KP ramps down proportionally
     *     so the robot decelerates smoothly as it approaches the target
     *   - D term damps oscillation by opposing rapid error changes
     *   - Deadband zeroes output for tiny errors to prevent micro-jitter
     *   - Output is clamped to MAX_ANGULAR_SPEED_RAD
     * 
     * Uses AIM_DIRECTION to flip the sign if the Pigeon mount compensation
     * isn't working correctly. If the robot rotates AWAY from the target,
     * flip AIM_DIRECTION to -1.0 in Constants.
     */
    private double aimOmega(double errorDeg) {
        // Deadband: zero output when error is tiny to prevent micro-oscillation
        if (Math.abs(errorDeg) < Constants.Driver.AIM_DEADBAND_DEG) {
            previousAimErrorRad = 0.0;
            return 0.0;
        }

        double errorRad = Math.toRadians(errorDeg);
        double absErrorDeg = Math.abs(errorDeg);

        // Slow-approach zone: scale KP down as we get closer to the target.
        // Outside the zone → full KP. At the deadband edge → KP scaled to near-minimum.
        // This creates a smooth deceleration ramp instead of an abrupt slowdown.
        double kp = Constants.Driver.AIM_HEADING_KP;
        double slowZone = Constants.Driver.AIM_SLOW_ZONE_DEG;
        if (absErrorDeg < slowZone) {
            // Scale from 1.0 (at zone edge) down to ~0.3 (near deadband)
            double blend = (absErrorDeg - Constants.Driver.AIM_DEADBAND_DEG)
                         / (slowZone - Constants.Driver.AIM_DEADBAND_DEG);
            blend = Math.max(0.0, Math.min(1.0, blend));
            // Minimum 30% KP so we still have enough authority to reach the target
            kp *= 0.3 + 0.7 * blend;
        }

        // P term: proportional to current error (with slow-zone scaling)
        double pTerm = errorRad * kp;

        // D term: opposes rapid changes in error (damps oscillation)
        double errorRate = (errorRad - previousAimErrorRad) / 0.020;
        double dTerm = errorRate * Constants.Driver.AIM_HEADING_KD;
        previousAimErrorRad = errorRad;

        double omega = (pTerm + dTerm) * Constants.Driver.AIM_DIRECTION;

        // Clamp to max rotation speed so large errors don't command unsafe rates
        double maxOmega = Constants.Driver.MAX_ANGULAR_SPEED_RAD;
        return Math.max(-maxOmega, Math.min(maxOmega, omega));
    }

    private void notify(String title) {
        Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle(title)
                .withDisplaySeconds(1.5));
    }
}
