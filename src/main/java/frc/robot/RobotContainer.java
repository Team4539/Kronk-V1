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
import frc.robot.commands.intake.RetractIntakeCommand;
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

    // Controllers
    private final CommandXboxController driver =
            new CommandXboxController(Constants.Driver.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
            new CommandXboxController(Constants.Driver.OPERATOR_CONTROLLER_PORT);

    // Swerve requests
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Driver.MAX_DRIVE_SPEED_MPS * Constants.Driver.STICK_DEADBAND)
            .withRotationalDeadband(Constants.Driver.MAX_ANGULAR_SPEED_RAD * Constants.Driver.STICK_DEADBAND)
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
        NamedCommands.registerCommand("wait0.5", Commands.waitSeconds(0.5));
        NamedCommands.registerCommand("wait1.0", Commands.waitSeconds(1.0));
        NamedCommands.registerCommand("wait2.0", Commands.waitSeconds(2.0));
    }

    // =========================================================================
    // CONTROLLER BINDINGS
    // =========================================================================

    private void configureBindings() {
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        // LT: X-lock brake
        if (drivetrain != null) {
            driver.leftTrigger(0.5).whileTrue(drivetrain.applyRequest(() -> brake));
        }

        // RB: Intake deploy/retract
        if (intake != null) {
            driver.rightBumper().onTrue(new DeployIntakeCommand(intake)
                    .alongWith(ledAction(LEDSubsystem.ActionState.INTAKING)));
            driver.rightBumper().onFalse(new RetractIntakeCommand(intake)
                    .alongWith(ledClear()));
        }

        // LB: Toggle field/robot centric
        driver.leftBumper().onTrue(Commands.runOnce(() -> {
            useRobotCentric = !useRobotCentric;
            notify(useRobotCentric ? "Robot Centric" : "Field Centric");
        }));

        // A: Auto-shoot
        if (shooter != null && vision != null) {
            driver.a().whileTrue(
                    new AutoShootCommand(shooter, vision, leds, trigger, drivetrain));
        }

        // B: Force shoot toggle
        driver.b().onTrue(Commands.runOnce(() -> {
            boolean on = !gameState.isForceShootEnabled();
            gameState.setForceShootEnabled(on);
            notify(on ? "Force Shoot ON" : "Force Shoot OFF");
        }));

        // Y / Start: Reset gyro
        if (drivetrain != null) {
            driver.y().onTrue(Commands.runOnce(() -> {
                drivetrain.seedFieldCentric();
                notify("Gyro Reset");
            }));
            driver.start().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));
        }

        // POV Up: Point wheels forward
        if (drivetrain != null) {
            driver.povUp().whileTrue(
                    drivetrain.applyRequest(() -> pointWheels.withModuleDirection(Rotation2d.kZero)));
        }
    }

    private void configureOperatorBindings() {
        // RT: Auto-shoot
        if (shooter != null && vision != null) {
            operator.rightTrigger(0.5).whileTrue(
                    new AutoShootCommand(shooter, vision, leds, trigger, drivetrain));
        }

        // LT: Pre-spool shooter
        if (shooter != null) {
            operator.leftTrigger(0.5).whileTrue(
                    Commands.run(() -> {
                        shooter.setTargetRPM(shootingCalc.getTargetRPM());
                        if (leds != null) leds.setAction(LEDSubsystem.ActionState.SPOOLING);
                    }, shooter).finallyDo(() -> {
                        shooter.stop();
                        if (leds != null) leds.clearAction();
                    }));
        }

        // RB: Shuttle mode toggle
        operator.rightBumper().onTrue(Commands.runOnce(() -> {
            boolean on = !gameState.isShuttleMode();
            gameState.setShuttleMode(on);
            notify(on ? "Shuttle Mode ON" : "Hub Mode");
        }));

        // LB: Force shoot toggle
        operator.leftBumper().onTrue(Commands.runOnce(() -> {
            boolean on = !gameState.isForceShootEnabled();
            gameState.setForceShootEnabled(on);
            notify(on ? "Force Shoot ON" : "Force Shoot OFF");
        }));

        // A: Intake
        if (intake != null) {
            operator.a().onTrue(new DeployIntakeCommand(intake)
                    .alongWith(ledAction(LEDSubsystem.ActionState.INTAKING)));
            operator.a().onFalse(new RetractIntakeCommand(intake)
                    .alongWith(ledClear()));
        }

        // X: E-stop motors
        operator.x().onTrue(Commands.runOnce(() -> {
            if (shooter != null) shooter.stop();
            if (trigger != null) trigger.stop();
            notify("E-STOP");
        }));

        // Start: Reset game state
        operator.start().onTrue(Commands.runOnce(() -> {
            gameState.reset();
            notify("Game State Reset");
        }));

        // Back: Clear shuttle override
        operator.back().onTrue(Commands.runOnce(() -> {
            gameState.clearShuttleModeOverride();
            notify("Auto-Shuttle Enabled");
        }));

        // POV Down: Feed shot (for calibration - feeds ball into shooter while held)
        if (trigger != null) {
            operator.povDown().whileTrue(
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
        // Full spool-up only happens when operator holds LT or during AutoShoot.
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
            shootingCalc.update(
                    drivetrain.getState().Pose,
                    drivetrain.getState().Speeds,
                    gameState.getTargetMode(),
                    calibration.getRPMOffset());
            calibration.setCurrentDistance(shootingCalc.getDistance());
            calibration.setCurrentBearing(shootingCalc.getRawBearing());
            calibration.setCurrentX(shootingCalc.getRelativeX());
            calibration.setCurrentY(shootingCalc.getRelativeY());
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

        // Fuse vision into drivetrain odometry.
        if (vision.hasVisionPose()) {
            edu.wpi.first.math.geometry.Pose2d visionPose = vision.getRobotPose();
            
            if (vision.isMultiTag()) {
                if (!hasSeededHeadingFromVision) {
                    drivetrain.resetPose(visionPose);
                    hasSeededHeadingFromVision = true;
                    vision.notifyHeadingSeeded();
                    System.out.println("[RobotContainer] Vision pose initialized (MultiTag): "
                            + String.format("(%.2f, %.2f) @ %.1f\u00b0",
                                    visionPose.getX(), visionPose.getY(),
                                    visionPose.getRotation().getDegrees()));
                    Elastic.sendNotification(
                            new Elastic.Notification(NotificationLevel.INFO,
                                    "Vision Pose Initialized",
                                    String.format("MultiTag: (%.1f, %.1f) @ %.1f\u00b0 from %d tags",
                                            visionPose.getX(), visionPose.getY(),
                                            visionPose.getRotation().getDegrees(),
                                            vision.getTargetCount())));
                }

                double[] std = vision.getVisionStdDevs();
                drivetrain.addVisionMeasurement(
                        visionPose,
                        vision.getPoseTimestamp(),
                        VecBuilder.fill(std[0], std[1], std[2]));
            } else {
                double[] std = vision.getVisionStdDevs();
                double looseTheta = 999.0;
                drivetrain.addVisionMeasurement(
                        visionPose,
                        vision.getPoseTimestamp(),
                        VecBuilder.fill(std[0], std[1], looseTheta));
                        
                if (!hasSeededHeadingFromVision) {
                    hasSeededHeadingFromVision = true;
                    vision.notifyHeadingSeeded();
                    System.out.println("[RobotContainer] Vision X/Y initialized (SingleTag, gyro heading kept): "
                            + String.format("(%.2f, %.2f)", visionPose.getX(), visionPose.getY()));
                    Elastic.sendNotification(
                            new Elastic.Notification(NotificationLevel.INFO,
                                    "Vision X/Y Initialized",
                                    String.format("SingleTag ID %d: (%.1f, %.1f) - heading from gyro",
                                            vision.getTargetId(),
                                            visionPose.getX(), visionPose.getY())));
                }
            }
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
