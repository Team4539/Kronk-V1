package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.turret.AimTurretToPoseCommand;
import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import frc.robot.commands.turret.AimTurretToTagsCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.calibrations.CalibrateIntakePivotsCommand;
import frc.robot.commands.calibrations.CalibrateTurretGearRatioCommand;
import frc.robot.commands.calibrations.DistanceOffsetCalibrationCommand;
import frc.robot.commands.calibrations.FullShooterCalibrationCommand;
import frc.robot.commands.calibrations.LimelightCalibrationCommand;
import frc.robot.commands.calibrations.ShootingCalibrationCommand;
import frc.robot.commands.calibrations.TurretPIDCalibrationCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.calibrations.PathPlannerTrainingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretFeedSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;

/**
 * Central hub that wires subsystems, commands, and controller bindings together.
 * Button bindings are in configureBindings(). Enable/disable subsystems in Constants.SubsystemEnabled.
 */
public class RobotContainer {

  private final GameStateManager gameState = GameStateManager.getInstance();

  // Subsystems (null if disabled in Constants.SubsystemEnabled)
  private final CommandSwerveDrivetrain drivetrain;
  private final TurretSubsystem turret;
  private final LimelightSubsystem limelight;
  private final ShooterSubsystem shooter;
  private final SpindexerSubsystem spindexer;
  private final TurretFeedSubsystem turretFeed;
  @SuppressWarnings("unused")
  private final IntakeSubsystem intake;
  private final LEDSubsystem leds;
  private final CalibrationManager calibrationManager;

  // Controllers
  private final CommandXboxController driverController = 
      new CommandXboxController(Constants.Driver.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = 
      new CommandXboxController(Constants.Driver.OPERATOR_CONTROLLER_PORT);

  // Swerve requests
  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Driver.MAX_DRIVE_SPEED_MPS * Constants.Driver.STICK_DEADBAND)
      .withRotationalDeadband(Constants.Driver.MAX_ANGULAR_SPEED_RAD * Constants.Driver.STICK_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(Constants.Driver.MAX_DRIVE_SPEED_MPS * Constants.Driver.STICK_DEADBAND)
      .withRotationalDeadband(Constants.Driver.MAX_ANGULAR_SPEED_RAD * Constants.Driver.STICK_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();

  // Commands (separate instances needed since commands can only be bound to one trigger)
  private final AimTurretToPoseCommand aimToPoseDefaultCommand;
  private final AimTurretToPoseCommand aimToPoseButtonCommand;
  private final AutoShootCommand autoShootCommand;
  private final CalibrateIntakePivotsCommand calibrateIntakePivotsCommand;

  // State
  private boolean isSlowMode = false;
  private boolean useRobotCentric = false;
  private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    // Initialize subsystems based on Constants.SubsystemEnabled flags
    drivetrain = Constants.SubsystemEnabled.DRIVETRAIN ? TunerConstants.createDrivetrain() : null;
    turret = Constants.SubsystemEnabled.TURRET ? new TurretSubsystem() : null;
    limelight = Constants.SubsystemEnabled.LIMELIGHT ? new LimelightSubsystem() : null;
    shooter = Constants.SubsystemEnabled.SHOOTER ? new ShooterSubsystem() : null;
    spindexer = Constants.SubsystemEnabled.SPINDEXER ? new SpindexerSubsystem() : null;
    turretFeed = Constants.SubsystemEnabled.TURRET_FEED ? new TurretFeedSubsystem() : null;
    intake = Constants.SubsystemEnabled.INTAKE ? new IntakeSubsystem() : null;
    leds = Constants.SubsystemEnabled.LEDS ? new LEDSubsystem() : null;
    calibrationManager = CalibrationManager.getInstance();
    
    if (leds != null) {
      leds.setRobotContainer(this);
    }
    
    // Log enabled subsystems
    logSubsystemStatus();
    
    // Create command instances
    if (turret != null && limelight != null) {
      aimToPoseDefaultCommand = new AimTurretToPoseCommand(turret, limelight);
      aimToPoseButtonCommand = new AimTurretToPoseCommand(turret, limelight);
    } else {
      aimToPoseDefaultCommand = null;
      aimToPoseButtonCommand = null;
    }
    
    autoShootCommand = (turret != null && shooter != null && limelight != null)
        ? new AutoShootCommand(turret, shooter, limelight, leds, spindexer, turretFeed, drivetrain)
        : null;
    
    calibrateIntakePivotsCommand = (intake != null)
        ? new CalibrateIntakePivotsCommand(intake)
        : null;
    
    registerNamedCommands();
    
    autoChooser = AutoBuilder.isConfigured() 
        ? AutoBuilder.buildAutoChooser()
        : createEmptyAutoChooser();
    
    configureBindings();
    configureDefaultCommands();
    configureDashboard();
  }
  
  private void logSubsystemStatus() {
    System.out.println("[Subsystems] Drivetrain: " + (drivetrain != null ? "ON" : "OFF"));
    System.out.println("[Subsystems] Turret: " + (turret != null ? "ON" : "OFF"));
    System.out.println("[Subsystems] Limelight: " + (limelight != null ? "ON" : "OFF"));
    System.out.println("[Subsystems] Shooter: " + (shooter != null ? "ON" : "OFF"));
    System.out.println("[Subsystems] Spindexer: " + (spindexer != null ? "ON" : "OFF"));
    System.out.println("[Subsystems] TurretFeed: " + (turretFeed != null ? "ON" : "OFF"));
    System.out.println("[Subsystems] Intake: " + (intake != null ? "ON" : "OFF"));
    System.out.println("[Subsystems] LEDs: " + (leds != null ? "ON" : "OFF"));
  }
  
  private SendableChooser<Command> createEmptyAutoChooser() {
    DriverStation.reportWarning("AutoBuilder not configured - check deploy/pathplanner/settings.json", false);
    return new SendableChooser<>();
  }

  // PathPlanner Named Commands
  
  private void registerNamedCommands() {
    // Shooting
    if (turret != null && shooter != null && limelight != null) {
      NamedCommands.registerCommand("autoShoot", 
          new AutoShootCommand(turret, shooter, limelight, leds, spindexer, turretFeed, drivetrain).withTimeout(3.0));
      NamedCommands.registerCommand("quickShoot", 
          new AutoShootCommand(turret, shooter, limelight, leds, spindexer, turretFeed, drivetrain).withTimeout(1.5));
    }
    
    // Aiming
    if (turret != null && limelight != null) {
      NamedCommands.registerCommand("aimAtPose", new AimTurretToPoseCommand(turret, limelight).withTimeout(2.0));
      NamedCommands.registerCommand("aimAtTags", new AimTurretToTagsCommand(turret, limelight).withTimeout(2.0));
    }
    
    // Shooter control
    if (shooter != null && limelight != null) {
      NamedCommands.registerCommand("spinUpShooter", Commands.run(() -> {
        double distance = limelight.hasPoseEstimate() ? limelight.getDistanceToHub() : 3.0;
        shooter.setForDistance(distance);
      }, shooter));
      NamedCommands.registerCommand("stopShooter", Commands.runOnce(() -> shooter.stop(), shooter));
    }
    
    // Turret
    if (turret != null) {
      NamedCommands.registerCommand("centerTurret", Commands.runOnce(() -> turret.setTargetAngle(0), turret));
    }
    
    // Game state
    NamedCommands.registerCommand("enableShuttleMode", Commands.runOnce(() -> gameState.setShuttleMode(true)));
    NamedCommands.registerCommand("disableShuttleMode", Commands.runOnce(() -> gameState.setShuttleMode(false)));
    
    // Utility
    NamedCommands.registerCommand("wait0.5", Commands.waitSeconds(0.5));
    NamedCommands.registerCommand("wait1.0", Commands.waitSeconds(1.0));
    NamedCommands.registerCommand("wait2.0", Commands.waitSeconds(2.0));
    NamedCommands.registerCommand("printMarker", Commands.print("PathPlanner Marker"));
    // TrainingShot: PathPlanner training stop - dashboard-only adjustments, no controller
    if (turret != null && shooter != null && limelight != null) {
      NamedCommands.registerCommand("TrainingShot", 
          new PathPlannerTrainingCommand(turret, shooter, limelight));
    } else {
      NamedCommands.registerCommand("TrainingShot", Commands.none());
    }
  }

  // Control Bindings
  
  private void configureBindings() {
    // === DRIVER CONTROLS ===
    
    // Right Bumper: Toggle slow mode
    driverController.rightBumper().onTrue(Commands.runOnce(() -> isSlowMode = !isSlowMode));
    
    // Left Bumper: Toggle field/robot centric
    driverController.leftBumper().onTrue(Commands.runOnce(() -> useRobotCentric = !useRobotCentric));
    
    if (drivetrain != null) {
      // Start: Reset gyro heading
      driverController.start().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));
      
      // Back: X-lock wheels
      driverController.back().whileTrue(drivetrain.applyRequest(() -> brakeRequest));
      
      // D-Pad Up: Point wheels forward
      driverController.povUp().whileTrue(
          drivetrain.applyRequest(() -> pointRequest.withModuleDirection(Rotation2d.kZero)));
    }
    
    // === OPERATOR CONTROLS ===
    
    // A: Auto-shoot
    if (autoShootCommand != null) {
      operatorController.a().whileTrue(autoShootCommand);
    }
    
    // B: Pose-based aiming only
    if (aimToPoseButtonCommand != null) {
      operatorController.b().whileTrue(aimToPoseButtonCommand);
    }
    
    // X: Intake deploy/stow
    if (intake != null) {
      operatorController.x().whileTrue(new DeployIntakeCommand(intake));
      operatorController.x().onFalse(new RetractIntakeCommand(intake));
    }

    
    
    // Y: Shooting calibration
    if (shooter != null && limelight != null) {
      operatorController.y().whileTrue(new ShootingCalibrationCommand(shooter, limelight));
    }
    
    // Right Trigger: Manual shooter spin-up
    if (shooter != null && limelight != null) {
      operatorController.rightTrigger(0.5).whileTrue(
          Commands.run(() -> {
            double distance = limelight.hasPoseEstimate() ? limelight.getDistanceToHub() : 3.0;
            shooter.setForDistance(distance);
          }, shooter).finallyDo(() -> shooter.stop()));
    }
    
    // Left Trigger: Emergency stop shooter
    if (shooter != null) {
      operatorController.leftTrigger(0.5).onTrue(Commands.runOnce(() -> shooter.stop()));
    }
    
    // Right Bumper: Toggle shuttle mode
    operatorController.rightBumper().onTrue(Commands.runOnce(() -> {
      boolean newMode = !gameState.isShuttleMode();
      gameState.setShuttleMode(newMode);
      System.out.println("Shuttle Mode: " + (newMode ? "ON" : "OFF"));
      Elastic.sendNotification(new Elastic.Notification()
          .withLevel(NotificationLevel.INFO)
          .withTitle(newMode ? " Shuttle Mode ON" : " Hub Mode ON")
          .withDescription(newMode ? "Aiming at trench" : "Aiming at hub")
          .withDisplaySeconds(2.0));
    }));

    // D-Pad Down: Clear shuttle mode override
    operatorController.povDown().onTrue(Commands.runOnce(() -> {
      gameState.clearShuttleModeOverride();
      System.out.println("Auto-shuttle re-enabled");
      Elastic.sendNotification(new Elastic.Notification()
          .withLevel(NotificationLevel.INFO)
          .withTitle("Auto-Shuttle Enabled")
          .withDescription("Auto-switching based on position")
          .withDisplaySeconds(2.0));
    }));
    
    // Left Bumper: Toggle force shoot
    operatorController.leftBumper().onTrue(Commands.runOnce(() -> {
      boolean newForce = !gameState.isForceShootEnabled();
      gameState.setForceShootEnabled(newForce);
      System.out.println("Force Shoot: " + (newForce ? "ON" : "OFF"));
      Elastic.sendNotification(new Elastic.Notification()
          .withLevel(newForce ? NotificationLevel.WARNING : NotificationLevel.INFO)
          .withTitle(newForce ? " Force Shoot ON" : "Force Shoot OFF")
          .withDescription(newForce ? "Ignoring alliance timing!" : "Normal timing restored")
          .withDisplaySeconds(3.0));
    }));
    
    // Start: Turret gear ratio calibration
  
    
    // Back: Reset game state
    operatorController.back().onTrue(Commands.runOnce(() -> {
      gameState.reset();
      System.out.println("Game State Reset");
      Elastic.sendNotification(new Elastic.Notification()
          .withLevel(NotificationLevel.INFO)
          .withTitle("Game State Reset")
          .withDescription("All modes cleared")
          .withDisplaySeconds(2.0));
    }));
  }

  // Default Commands
  
  // Default Commands
  
  private void configureDefaultCommands() {
    // Drivetrain: Swerve drive with joystick input
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
        if (!gameState.shouldDriveRun()) {
          return fieldCentricDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        }
        
        double speedMultiplier = (isSlowMode ? Constants.Driver.SLOW_MODE_MULTIPLIER : 1.0) 
            * gameState.getEffectiveSpeedMultiplier();
        
        double xSpeed = -driverController.getLeftY() * Constants.Driver.MAX_DRIVE_SPEED_MPS * speedMultiplier;
        double ySpeed = -driverController.getLeftX() * Constants.Driver.MAX_DRIVE_SPEED_MPS * speedMultiplier;
        double rotSpeed = -driverController.getRightX() * Constants.Driver.MAX_ANGULAR_SPEED_RAD * speedMultiplier;
        
        if (useRobotCentric) {
          return robotCentricDrive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rotSpeed);
        }
        return fieldCentricDrive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rotSpeed);
      }));
    }
    
    // Turret: Auto-aim when idle
    if (turret != null && aimToPoseDefaultCommand != null) {
      turret.setDefaultCommand(aimToPoseDefaultCommand);
    }
    
    // Shooter: Default to 87% power to stay ready to shoot
    if (shooter != null) {
      shooter.setDefaultCommand(Commands.run(() -> {
        shooter.setManualPower(Constants.Shooter.DEFAULT_IDLE_POWER, Constants.Shooter.DEFAULT_IDLE_POWER);
      }, shooter));
    }
  }

  // Dashboard
  
  private void configureDashboard() {
    DashboardHelper.putData(Category.PRE_MATCH, "Auto Chooser", autoChooser);
    
    // Calibration commands
    if (turret != null && shooter != null && limelight != null) {
      DashboardHelper.putData(Category.SETTINGS, "Cal/FullShooter", new FullShooterCalibrationCommand(turret, shooter, limelight));
    }
    if (turret != null) {
      DashboardHelper.putData(Category.SETTINGS, "Cal/TurretGearRatio", new CalibrateTurretGearRatioCommand(turret));
      DashboardHelper.putData(Category.SETTINGS, "Cal/TurretPID", new TurretPIDCalibrationCommand(turret));
    }
    if (limelight != null) {
      DashboardHelper.putData(Category.SETTINGS, "Cal/Limelight", new LimelightCalibrationCommand(limelight));
    }
    if (shooter != null && limelight != null) {
      DashboardHelper.putData(Category.SETTINGS, "Cal/DistanceOffset", new DistanceOffsetCalibrationCommand(shooter, limelight));
      DashboardHelper.putData(Category.SETTINGS, "Cal/Shooting", new ShootingCalibrationCommand(shooter, limelight));
    }
    
    // Aim mode commands
    if (turret != null && limelight != null) {
      DashboardHelper.putData(Category.SETTINGS, "AimMode/Pose", new AimTurretToPoseCommand(turret, limelight));
      DashboardHelper.putData(Category.SETTINGS, "AimMode/Tags", new AimTurretToTagsCommand(turret, limelight));
    }
    if (autoShootCommand != null) {
      DashboardHelper.putData(Category.SETTINGS, "AutoShoot", autoShootCommand);
    }
    
    // Status
    DashboardHelper.putBoolean(Category.PRE_MATCH, "Subsystems/Drivetrain", drivetrain != null);
    DashboardHelper.putBoolean(Category.PRE_MATCH, "Subsystems/Turret", turret != null);
    DashboardHelper.putBoolean(Category.PRE_MATCH, "Subsystems/Shooter", shooter != null);
    DashboardHelper.putBoolean(Category.PRE_MATCH, "Subsystems/Limelight", limelight != null);
  }

  // Periodic Updates
  
  /** Updates vision pose and game state. Call from Robot.robotPeriodic(). */
  public void updateVisionPose() {
    // Always update calibration manager regardless of subsystem state
    calibrationManager.update();
    
    if (limelight == null || drivetrain == null) return;
    
    limelight.setDrivetrainPose(drivetrain.getState().Pose);
    gameState.update(calculateShuttleZone());
    
    if (limelight.hasPoseEstimate()) {
      double[] stdDevs = limelight.getVisionStdDevs();
      drivetrain.addVisionMeasurement(
          limelight.getRobotPose(),
          limelight.getPoseTimestamp(),
          VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]));
    }
    
    DashboardHelper.putBoolean(Category.TELEOP, "Drive/SlowMode", isSlowMode);
    DashboardHelper.putBoolean(Category.TELEOP, "Drive/RobotCentric", useRobotCentric);
  }
  
  private boolean calculateShuttleZone() {
    if (drivetrain == null) return false;
    
    double robotX = drivetrain.getState().Pose.getX();
    double boundaryX = DashboardHelper.getNumber(Category.SETTINGS, "Aim/AutoShuttleLineX", Constants.Field.AUTO_SHUTTLE_BOUNDARY_X);
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    
    boolean inZone = (alliance == DriverStation.Alliance.Blue) 
        ? robotX > boundaryX 
        : robotX < (Constants.Field.FIELD_LENGTH_METERS - boundaryX);
    
    return inZone;
  }

  // Command Getters
  
  public Command getAimTurretCommand() {
    return (turret != null && limelight != null) 
        ? new AimTurretToTagsCommand(turret, limelight) 
        : Commands.none();
  }

  public Command getAimTurretToPoseCommand() {
    return (turret != null && limelight != null) 
        ? new AimTurretToPoseCommand(turret, limelight) 
        : Commands.none();
  }

  public Command getAutoShootCommand() {
    return autoShootCommand != null ? autoShootCommand : Commands.none();
  }

  /** Turret gear ratio calibration - rotate turret 360° by hand while command runs. */
  public Command getCalibrateTurretCommand() {
    return turret != null ? new CalibrateTurretGearRatioCommand(turret) : Commands.none();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  
  public LEDSubsystem getLEDs() {
    return leds;
  }

  public IntakeSubsystem getIntake() {
    return intake;
  }
  public LimelightSubsystem getLimelight() {
    return limelight;
  }
  public ShooterSubsystem getShooter() {
    return shooter;
  }
  public SpindexerSubsystem getSpindexer(){
    return spindexer;
  }
}
