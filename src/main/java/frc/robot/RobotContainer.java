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
import frc.robot.commands.calibrations.CalibrateTurretGearRatioCommand;
import frc.robot.commands.calibrations.DistanceOffsetCalibrationCommand;
import frc.robot.commands.calibrations.FullShooterCalibrationCommand;
import frc.robot.commands.calibrations.LimelightCalibrationCommand;
import frc.robot.commands.calibrations.ShootingCalibrationCommand;
import frc.robot.commands.calibrations.TurretPIDCalibrationCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.turret.AimTurretToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretFeedSubsystem;
import frc.robot.subsystems.TurretSubsystem;
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
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final LimelightSubsystem limelight;
    private final TurretFeedSubsystem turretFeed;
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

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================

    public RobotContainer() {
        // 1. Init subsystems
        drivetrain = Constants.SubsystemEnabled.DRIVETRAIN  ? TunerConstants.createDrivetrain() : null;
        turret     = Constants.SubsystemEnabled.TURRET      ? new TurretSubsystem()             : null;
        shooter    = Constants.SubsystemEnabled.SHOOTER     ? new ShooterSubsystem()             : null;
        limelight  = Constants.SubsystemEnabled.LIMELIGHT   ? new LimelightSubsystem()           : null;
        turretFeed = Constants.SubsystemEnabled.TURRET_FEED ? new TurretFeedSubsystem()          : null;
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
                + " Turret=" + (turret != null) + " Shooter=" + (shooter != null)
                + " Limelight=" + (limelight != null) + " Feed=" + (turretFeed != null)
                + " Intake=" + (intake != null) + " LEDs=" + (leds != null));
    }

    // =========================================================================
    // PATHPLANNER NAMED COMMANDS
    // =========================================================================

    private void registerNamedCommands() {
        if (turret != null && shooter != null && limelight != null) {
            NamedCommands.registerCommand("autoShoot",
                    new AutoShootCommand(turret, shooter, limelight, leds, turretFeed, drivetrain)
                            .withTimeout(3.0));
            NamedCommands.registerCommand("quickShoot",
                    new AutoShootCommand(turret, shooter, limelight, leds, turretFeed, drivetrain)
                            .withTimeout(1.5));
        }
        if (turret != null && limelight != null) {
            NamedCommands.registerCommand("aimAtPose",
                    new AimTurretToPoseCommand(turret, limelight).withTimeout(2.0));
        }
        if (shooter != null) {
            NamedCommands.registerCommand("spinUpShooter",
                    Commands.run(() -> shooter.setTargetRPM(
                            shootingCalc.getTargetTopRPM(),
                            shootingCalc.getTargetBottomRPM()), shooter));
            NamedCommands.registerCommand("stopShooter",
                    Commands.runOnce(() -> shooter.stop(), shooter));
        }
        if (turret != null) {
            NamedCommands.registerCommand("centerTurret",
                    Commands.runOnce(() -> turret.setTargetAngle(0), turret));
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
        if (turret != null && shooter != null && limelight != null) {
            driver.a().whileTrue(
                    new AutoShootCommand(turret, shooter, limelight, leds, turretFeed, drivetrain));
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
        if (turret != null && shooter != null && limelight != null) {
            operator.rightTrigger(0.5).whileTrue(
                    new AutoShootCommand(turret, shooter, limelight, leds, turretFeed, drivetrain));
        }

        // LT: Pre-spool shooter
        if (shooter != null) {
            operator.leftTrigger(0.5).whileTrue(
                    Commands.run(() -> {
                        shooter.setTargetRPM(shootingCalc.getTargetTopRPM(), shootingCalc.getTargetBottomRPM());
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

        // B: Aim turret only
        if (turret != null && limelight != null) {
            operator.b().whileTrue(new AimTurretToPoseCommand(turret, limelight));
        }

        // X: E-stop motors
        operator.x().onTrue(Commands.runOnce(() -> {
            if (shooter != null) shooter.stop();
            if (turretFeed != null) turretFeed.stop();
            notify("E-STOP");
        }));

        // Y: Center turret
        if (turret != null) {
            operator.y().onTrue(Commands.runOnce(() -> turret.setTargetAngle(0), turret));
        }

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
    }

    // =========================================================================
    // DEFAULT COMMANDS
    // =========================================================================

    private void configureDefaultCommands() {
        // Drivetrain: swerve drive with slow-mode on RT
        if (drivetrain != null) {
            drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
                double slow = driver.getRightTriggerAxis() > 0.3
                        ? Constants.Driver.SLOW_MODE_MULTIPLIER : 1.0;
                double vx = -driver.getLeftY()  * Constants.Driver.MAX_DRIVE_SPEED_MPS  * slow;
                double vy = -driver.getLeftX()  * Constants.Driver.MAX_DRIVE_SPEED_MPS  * slow;
                double vr = -driver.getRightX() * Constants.Driver.MAX_ANGULAR_SPEED_RAD * slow;

                if (useRobotCentric) {
                    return robotCentric.withVelocityX(vx).withVelocityY(vy).withRotationalRate(vr);
                }
                return fieldCentric.withVelocityX(vx).withVelocityY(vy).withRotationalRate(vr);
            }));
        }

        // Turret: auto-aim at current target
        if (turret != null && limelight != null) {
            turret.setDefaultCommand(new AimTurretToPoseCommand(turret, limelight));
        }

        // Shooter: idle pre-spool at calculated RPM (only when target mode is active)
        // When DISABLED, ShootingCalculator already outputs 0 RPM, so shooter idles naturally.
        // This avoids wasting battery/heating motors when we can't score.
        if (shooter != null) {
            shooter.setDefaultCommand(Commands.run(() -> {
                double topRPM = shootingCalc.getTargetTopRPM();
                double bottomRPM = shootingCalc.getTargetBottomRPM();
                if (topRPM > 0.1 || bottomRPM > 0.1) {
                    shooter.setTargetRPM(topRPM, bottomRPM);
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
        if (turret != null && shooter != null && limelight != null) {
            SmartDashboard.putData("Tuning/Cal: Full Shooter",
                    new FullShooterCalibrationCommand(turret, shooter, limelight, turretFeed));
        }
        if (shooter != null && limelight != null) {
            SmartDashboard.putData("Tuning/Cal: Shooting",
                    new ShootingCalibrationCommand(shooter, limelight));
            SmartDashboard.putData("Tuning/Cal: Distance Offset",
                    new DistanceOffsetCalibrationCommand(shooter, limelight));
        }
        if (turret != null) {
            SmartDashboard.putData("Tuning/Cal: Turret Gear Ratio",
                    new CalibrateTurretGearRatioCommand(turret));
            SmartDashboard.putData("Tuning/Cal: Turret PID",
                    new TurretPIDCalibrationCommand(turret));
        }
        if (limelight != null) {
            SmartDashboard.putData("Tuning/Cal: Limelight",
                    new LimelightCalibrationCommand(limelight));
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
                    calibration.getTurretAngleOffset());
            calibration.setCurrentDistance(shootingCalc.getDistance());
        }

        if (limelight == null || drivetrain == null) return;

        // Feed drivetrain pose to limelight for MegaTag
        limelight.setDrivetrainPose(drivetrain.getState().Pose);

        // Auto-shuttle zone check
        gameState.update(isInShuttleZone());

        // Fuse vision into drivetrain odometry
        if (limelight.hasVisionPose()) {
            double[] std = limelight.getVisionStdDevs();
            drivetrain.addVisionMeasurement(
                    limelight.getRobotPose(),
                    limelight.getPoseTimestamp(),
                    VecBuilder.fill(std[0], std[1], std[2]));
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
        if (shooter != null)    shooter.stop();
        if (turretFeed != null) turretFeed.stop();
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
