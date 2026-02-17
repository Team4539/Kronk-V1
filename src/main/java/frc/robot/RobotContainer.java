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
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.calibrations.CalibrateTurretGearRatioCommand;
import frc.robot.commands.calibrations.DistanceOffsetCalibrationCommand;
import frc.robot.commands.calibrations.FullShooterCalibrationCommand;
import frc.robot.commands.calibrations.LimelightCalibrationCommand;
import frc.robot.commands.calibrations.ShootingCalibrationCommand;
import frc.robot.commands.calibrations.TurretPIDCalibrationCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
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
import frc.robot.util.PiShootingHelper;

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
  private final TurretFeedSubsystem turretFeed;
  private final PiShootingHelper piShootingHelper;
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

  // State
  private boolean useRobotCentric = false;
  private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    // Initialize subsystems based on Constants.SubsystemEnabled flags
    drivetrain = Constants.SubsystemEnabled.DRIVETRAIN ? TunerConstants.createDrivetrain() : null;
    turret = Constants.SubsystemEnabled.TURRET ? new TurretSubsystem() : null;
    limelight = Constants.SubsystemEnabled.LIMELIGHT ? new LimelightSubsystem() : null;
    shooter = Constants.SubsystemEnabled.SHOOTER ? new ShooterSubsystem() : null;
    turretFeed = Constants.SubsystemEnabled.TURRET_FEED ? new TurretFeedSubsystem() : null;
    intake = Constants.SubsystemEnabled.INTAKE ? new IntakeSubsystem() : null;
    leds = Constants.SubsystemEnabled.LEDS ? new LEDSubsystem() : null;
    calibrationManager = CalibrationManager.getInstance();
    piShootingHelper = PiShootingHelper.getInstance();
    
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
        ? new AutoShootCommand(turret, shooter, limelight, leds, turretFeed, drivetrain)
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
          new AutoShootCommand(turret, shooter, limelight, leds, turretFeed, drivetrain).withTimeout(3.0));
      NamedCommands.registerCommand("quickShoot", 
          new AutoShootCommand(turret, shooter, limelight, leds, turretFeed, drivetrain).withTimeout(1.5));
    }
    
    // Aiming
    if (turret != null && limelight != null) {
      NamedCommands.registerCommand("aimAtPose", new AimTurretToPoseCommand(turret, limelight).withTimeout(2.0));
    }
    
    // Shooter control
    if (shooter != null && limelight != null) {
      NamedCommands.registerCommand("spinUpShooter", Commands.run(() -> {
        // Use Pi shooting helper for shooter power - it has the full shooting solution
        PiShootingHelper piHelper = PiShootingHelper.getInstance();
        double topPower = piHelper.getTopSpeed();
        double bottomPower = piHelper.getBottomSpeed();
        // If Pi hasn't calculated yet (both zero), use default idle power
        if (topPower == 0.0 && bottomPower == 0.0) {
          topPower = Constants.Shooter.DEFAULT_IDLE_POWER;
          bottomPower = Constants.Shooter.DEFAULT_IDLE_POWER;
        }
        shooter.setManualPower(topPower, bottomPower);
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
  }

  // Control Bindings
  
  private void configureBindings() {
    // +==================================================================+
    // |                      DRIVER CONTROLLER                           |
    // |  Left Stick  = Drive X/Y      Right Stick X = Rotation           |
    // |  RT (hold)   = Slow mode      LT (hold)     = X-lock brake       |
    // |  RB (hold)   = Intake         LB            = Field/Robot tog    |
    // |  A  (hold)   = Auto-shoot     Y             = Reset gyro         |
    // |  B           = Force shoot    X             = (spare)            |
    // |  Start       = Reset gyro     Back          = (spare)            |
    // |  POV Up      = Point wheels   POV Down      = Seed field centr   |
    // +==================================================================+
    // |                     OPERATOR CONTROLLER                          |
    // |  RT (hold)   = Auto-shoot     LT (hold)     = Pre-spool          | 
    // |  RB          = Shuttle toggle LB            = Force shoot tog    |
    // |  A  (hold)   = Intake         B  (hold)     = Aim turret only    |
    // |  X           = E-stop motors  Y             = Center turret      |
    // |  Start       = Reset game     Back          = Clear shuttle ovr  |
    // |  POV Up      = HIT (train)    POV Right     = MISS (train)       |
    // |  POV Left    = Discard (train)POV Down      = (spare)            |
    // +==================================================================+
    
    // === DRIVER CONTROLS ===
    // Philosophy: Driver drives. Most-used actions on triggers (ergonomic holds).
    // Slow mode is HOLD (not toggle) -- more intuitive and safer mid-match.
    
    // RT (hold): Slow mode -- hold for precision, release for full speed
    // (Handled in default command via trigger axis -- no binding needed here)
    
    // LT (hold): X-lock brake -- instant defensive stop
    if (drivetrain != null) {
      driverController.leftTrigger(0.5).whileTrue(drivetrain.applyRequest(() -> brakeRequest));
    }
    
    // RB (hold): Intake -- driver deploys intake since they see the game pieces
    if (intake != null) {
      driverController.rightBumper().onTrue(
          new DeployIntakeCommand(intake)
              .alongWith(Commands.runOnce(() -> { if (leds != null) leds.setAction(LEDSubsystem.ActionState.INTAKING); })));
      driverController.rightBumper().onFalse(
          new RetractIntakeCommand(intake)
              .alongWith(Commands.runOnce(() -> { if (leds != null) leds.clearAction(); })));
    }
    
    // LB: Toggle field-centric / robot-centric (rarely used, bumper is fine)
    driverController.leftBumper().onTrue(Commands.runOnce(() -> {
      useRobotCentric = !useRobotCentric;
      Elastic.sendNotification(new Elastic.Notification()
          .withLevel(NotificationLevel.INFO)
          .withTitle(useRobotCentric ? "Robot Centric" : "Field Centric")
          .withDescription(useRobotCentric ? "Driving relative to robot front" : "Driving relative to field")
          .withDisplaySeconds(1.5));
    }));
    
    // A (hold): Auto-shoot -- driver can trigger shooting for fast gameplay
    if (autoShootCommand != null) {
      // Need a separate command instance since driver + operator can't share
      AutoShootCommand driverAutoShoot = (turret != null && shooter != null && limelight != null)
          ? new AutoShootCommand(turret, shooter, limelight, leds, turretFeed, drivetrain)
          : null;
      if (driverAutoShoot != null) {
        driverController.a().whileTrue(driverAutoShoot);
      }
    }
    
    // B: Toggle force shoot -- driver emergency override for scoring outside alliance window
    driverController.b().onTrue(Commands.runOnce(() -> {
      boolean newForce = !gameState.isForceShootEnabled();
      gameState.setForceShootEnabled(newForce);
      Elastic.sendNotification(new Elastic.Notification()
          .withLevel(newForce ? NotificationLevel.WARNING : NotificationLevel.INFO)
          .withTitle(newForce ? "! Force Shoot ON" : "Force Shoot OFF")
          .withDescription(newForce ? "Ignoring alliance timing!" : "Normal timing restored")
          .withDisplaySeconds(2.0));
    }));
    
    // Y: Reset gyro heading
    if (drivetrain != null) {
      driverController.y().onTrue(Commands.runOnce(() -> {
        drivetrain.seedFieldCentric();
        Elastic.sendNotification(new Elastic.Notification()
            .withLevel(NotificationLevel.INFO)
            .withTitle("Gyro Reset")
            .withDescription("Field-centric heading zeroed")
            .withDisplaySeconds(1.5));
      }));
    }
    
    // Start: Reset gyro (backup binding for muscle memory)
    if (drivetrain != null) {
      driverController.start().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));
    }
    
    // POV Up: Point wheels forward (useful for alignment)
    if (drivetrain != null) {
      driverController.povUp().whileTrue(
          drivetrain.applyRequest(() -> pointRequest.withModuleDirection(Rotation2d.kZero)));
    }
    
    // === OPERATOR CONTROLS ===
    // Philosophy: Operator manages weapons. Primary action (shooting) on RT.
    // Face buttons handle intake/aim/safety. D-pad reserved for training.
    
    // RT (hold): Auto-shoot -- the #1 operator action, on the most ergonomic hold button
    if (turret != null && shooter != null && limelight != null) {
      operatorController.rightTrigger(0.5).whileTrue(autoShootCommand);
    }
    
    // LT (hold): Pre-spool shooter -- spin up flywheels before our alliance window opens
    if (shooter != null && limelight != null) {
      operatorController.leftTrigger(0.5).whileTrue(
          Commands.run(() -> {
            PiShootingHelper piHelper = PiShootingHelper.getInstance();
            double topPower = piHelper.getTopSpeed();
            double bottomPower = piHelper.getBottomSpeed();
            if (topPower == 0.0 && bottomPower == 0.0) {
              topPower = Constants.Shooter.DEFAULT_IDLE_POWER;
              bottomPower = Constants.Shooter.DEFAULT_IDLE_POWER;
            }
            shooter.setManualPower(topPower, bottomPower);
            if (leds != null) leds.setAction(LEDSubsystem.ActionState.SPOOLING);
          }, shooter).finallyDo(() -> {
            shooter.stop();
            if (leds != null) leds.clearAction();
          }));
    }
    
    // RB: Toggle shuttle mode (hub <-> trench)
    operatorController.rightBumper().onTrue(Commands.runOnce(() -> {
      boolean newMode = !gameState.isShuttleMode();
      gameState.setShuttleMode(newMode);
      Elastic.sendNotification(new Elastic.Notification()
          .withLevel(NotificationLevel.INFO)
          .withTitle(newMode ? "Shuttle Mode ON" : "Hub Mode ON")
          .withDescription(newMode ? "Aiming at trench" : "Aiming at hub")
          .withDisplaySeconds(2.0));
    }));
    
    // LB: Toggle force shoot (operator can also toggle this)
    operatorController.leftBumper().onTrue(Commands.runOnce(() -> {
      boolean newForce = !gameState.isForceShootEnabled();
      gameState.setForceShootEnabled(newForce);
      Elastic.sendNotification(new Elastic.Notification()
          .withLevel(newForce ? NotificationLevel.WARNING : NotificationLevel.INFO)
          .withTitle(newForce ? "! Force Shoot ON" : "Force Shoot OFF")
          .withDescription(newForce ? "Ignoring alliance timing!" : "Normal timing restored")
          .withDisplaySeconds(3.0));
    }));
    
    // A (hold): Intake deploy/retract -- operator backup for intake control
    if (intake != null) {
      operatorController.a().onTrue(
          new DeployIntakeCommand(intake)
              .alongWith(Commands.runOnce(() -> { if (leds != null) leds.setAction(LEDSubsystem.ActionState.INTAKING); })));
      operatorController.a().onFalse(
          new RetractIntakeCommand(intake)
              .alongWith(Commands.runOnce(() -> { if (leds != null) leds.clearAction(); })));
    }
    
    // B (hold): Aim turret only (pre-aim without shooting -- useful during waits)
    if (aimToPoseButtonCommand != null) {
      operatorController.b().whileTrue(
          aimToPoseButtonCommand.beforeStarting(() -> { if (leds != null) leds.setAction(LEDSubsystem.ActionState.AIMING); })
              .finallyDo(() -> { if (leds != null) leds.clearAction(); }));
    }
    
    // X: Emergency stop all shooter/feed motors
    operatorController.x().onTrue(Commands.runOnce(() -> {
      if (shooter != null) shooter.stop();
      if (turretFeed != null) turretFeed.stop();
      Elastic.sendNotification(new Elastic.Notification()
          .withLevel(NotificationLevel.WARNING)
          .withTitle("E-STOP")
          .withDescription("Shooter & feed motors stopped")
          .withDisplaySeconds(2.0));
    }));
    
    // Y: Center turret (safety position -- turret faces forward)
    if (turret != null) {
      operatorController.y().onTrue(Commands.runOnce(() -> {
        turret.setTargetAngle(0);
        Elastic.sendNotification(new Elastic.Notification()
            .withLevel(NotificationLevel.INFO)
            .withTitle("Turret Centered")
            .withDescription("Turret reset to 0 deg")
            .withDisplaySeconds(1.5));
      }));
    }
    
    // Start: Reset game state (clears all mode overrides)
    operatorController.start().onTrue(Commands.runOnce(() -> {
      gameState.reset();
      Elastic.sendNotification(new Elastic.Notification()
          .withLevel(NotificationLevel.INFO)
          .withTitle("Game State Reset")
          .withDescription("All modes cleared")
          .withDisplaySeconds(2.0));
    }));
    
    // Back: Clear shuttle mode override (re-enable auto-switching)
    operatorController.back().onTrue(Commands.runOnce(() -> {
      gameState.clearShuttleModeOverride();
      Elastic.sendNotification(new Elastic.Notification()
          .withLevel(NotificationLevel.INFO)
          .withTitle("Auto-Shuttle Enabled")
          .withDescription("Auto-switching based on position")
          .withDisplaySeconds(2.0));
    }));
    
    // ---- TRAINING DATA RECORDING (Two-Phase) ----
    // The robot automatically snapshots state when a shot is fired.
    // Operator then confirms the result after watching the ball land.
    // POV Up = HIT, POV Right = MISS, POV Left = Discard bad data
    
    // POV Up: Confirm shot as HIT
    operatorController.povUp().onTrue(Commands.runOnce(() -> {
      if (piShootingHelper.hasPendingShot()) {
        boolean confirmed = piShootingHelper.confirmShot("HIT");
        if (confirmed) {
          Elastic.sendNotification(new Elastic.Notification()
              .withLevel(NotificationLevel.INFO)
              .withTitle("[OK] HIT Confirmed")
              .withDescription(String.format("Shot at %.1fm logged to training data", 
                  piShootingHelper.getDistance()))
              .withDisplaySeconds(2.0));
        }
      } else {
        Elastic.sendNotification(new Elastic.Notification()
            .withLevel(NotificationLevel.WARNING)
            .withTitle("No Pending Shot")
            .withDescription("Fire a shot first, then confirm HIT/MISS")
            .withDisplaySeconds(2.0));
      }
    }));
    
    // POV Right: Confirm shot as MISS
    operatorController.povRight().onTrue(Commands.runOnce(() -> {
      if (piShootingHelper.hasPendingShot()) {
        boolean confirmed = piShootingHelper.confirmShot("MISS");
        if (confirmed) {
          Elastic.sendNotification(new Elastic.Notification()
              .withLevel(NotificationLevel.INFO)
              .withTitle("[X] MISS Confirmed")
              .withDescription(String.format("Shot at %.1fm logged to training data", 
                  piShootingHelper.getDistance()))
              .withDisplaySeconds(2.0));
        }
      } else {
        Elastic.sendNotification(new Elastic.Notification()
            .withLevel(NotificationLevel.WARNING)
            .withTitle("No Pending Shot")
            .withDescription("Fire a shot first, then confirm HIT/MISS")
            .withDisplaySeconds(2.0));
      }
    }));
    
    // POV Left: Discard pending shot (bad data, don't log)
    operatorController.povLeft().onTrue(Commands.runOnce(() -> {
      if (piShootingHelper.hasPendingShot()) {
        piShootingHelper.discardPendingShot();
        Elastic.sendNotification(new Elastic.Notification()
            .withLevel(NotificationLevel.INFO)
            .withTitle("Shot Discarded")
            .withDescription("Pending shot data thrown away")
            .withDisplaySeconds(2.0));
      }
    }));
  }

  // Default Commands
  
  private void configureDefaultCommands() {
    // Drivetrain: Swerve drive with joystick input
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
        if (!gameState.shouldDriveRun()) {
          return fieldCentricDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        }
        
        // RT = slow mode (hold for precision). Trigger axis 0.0-1.0 maps to full speed -> slow speed.
        double triggerValue = driverController.getRightTriggerAxis();
        boolean slowActive = triggerValue > 0.3;
        double speedMultiplier = (slowActive ? Constants.Driver.SLOW_MODE_MULTIPLIER : 1.0) 
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
            PiShootingHelper piHelper = PiShootingHelper.getInstance();
            double topPower = piHelper.getTopSpeed();
            double bottomPower = piHelper.getBottomSpeed();
            // If Pi hasn't calculated yet (both zero), use default idle power
            if (topPower == 0.0 && bottomPower == 0.0) {
              topPower = Constants.Shooter.DEFAULT_IDLE_POWER;
              bottomPower = Constants.Shooter.DEFAULT_IDLE_POWER;
            }
            shooter.setManualPower(topPower, bottomPower);
      }, shooter));
    }
  }

  // Dashboard
  
  private void configureDashboard() {
    DashboardHelper.putData(Category.PRE_MATCH, "Auto Chooser", autoChooser);
    
    // ===== CALIBRATION COMMANDS (dashboard buttons) =====
    // All calibration commands are available here so no controller buttons are needed.
    
    // Full calibration routines
    if (turret != null && shooter != null && limelight != null) {
      DashboardHelper.putData(Category.SETTINGS, "Cal/FullShooter", new FullShooterCalibrationCommand(turret, shooter, limelight, turretFeed));
    }
    if (shooter != null && limelight != null) {
      DashboardHelper.putData(Category.SETTINGS, "Cal/Shooting", new ShootingCalibrationCommand(shooter, limelight));
      DashboardHelper.putData(Category.SETTINGS, "Cal/DistanceOffset", new DistanceOffsetCalibrationCommand(shooter, limelight));
    }
    if (turret != null) {
      DashboardHelper.putData(Category.SETTINGS, "Cal/TurretGearRatio", new CalibrateTurretGearRatioCommand(turret));
      DashboardHelper.putData(Category.SETTINGS, "Cal/TurretPID", new TurretPIDCalibrationCommand(turret));
    }
    if (limelight != null) {
      DashboardHelper.putData(Category.SETTINGS, "Cal/Limelight", new LimelightCalibrationCommand(limelight));
    }
    
    // Aiming & shooting commands
    if (turret != null && limelight != null) {
      DashboardHelper.putData(Category.SETTINGS, "AimMode/Pose", new AimTurretToPoseCommand(turret, limelight));
    }
    if (turret != null && shooter != null && limelight != null) {
      DashboardHelper.putData(Category.SETTINGS, "AutoShoot",
          new AutoShootCommand(turret, shooter, limelight, leds, turretFeed, drivetrain));
    }
    
    // Manual shooter controls
    if (shooter != null) {
      DashboardHelper.putData(Category.SETTINGS, "Shooter/SpinUp", Commands.run(() -> {
        PiShootingHelper piHelper = PiShootingHelper.getInstance();
        double topPower = piHelper.getTopSpeed();
        double bottomPower = piHelper.getBottomSpeed();
        if (topPower == 0.0 && bottomPower == 0.0) {
          topPower = Constants.Shooter.DEFAULT_IDLE_POWER;
          bottomPower = Constants.Shooter.DEFAULT_IDLE_POWER;
        }
        shooter.setManualPower(topPower, bottomPower);
      }, shooter));
      DashboardHelper.putData(Category.SETTINGS, "Shooter/Stop", Commands.runOnce(() -> shooter.stop(), shooter));
    }
    
    // Turret controls
    if (turret != null) {
      DashboardHelper.putData(Category.SETTINGS, "Turret/CenterTurret", Commands.runOnce(() -> turret.setTargetAngle(0), turret));
    }
    
    // Intake controls
    if (intake != null) {
      DashboardHelper.putData(Category.SETTINGS, "Intake/Deploy", new DeployIntakeCommand(intake));
      DashboardHelper.putData(Category.SETTINGS, "Intake/Retract", new RetractIntakeCommand(intake));
    }
    
    // Turret feed controls
    if (turretFeed != null) {
      DashboardHelper.putData(Category.SETTINGS, "TurretFeed/Feed", Commands.run(() -> turretFeed.setShoot(), turretFeed));
      DashboardHelper.putData(Category.SETTINGS, "TurretFeed/Stop", Commands.runOnce(() -> turretFeed.stop(), turretFeed));
    }
    
    // Drivetrain controls
    if (drivetrain != null) {
      DashboardHelper.putData(Category.SETTINGS, "Drive/ResetGyro", Commands.runOnce(() -> drivetrain.seedFieldCentric()));
      DashboardHelper.putData(Category.SETTINGS, "Drive/BrakeXLock", drivetrain.applyRequest(() -> brakeRequest));
    }
    
    // Status
    DashboardHelper.putBoolean(Category.PRE_MATCH, "Subsystems/Drivetrain", drivetrain != null);
    DashboardHelper.putBoolean(Category.PRE_MATCH, "Subsystems/Turret", turret != null);
    DashboardHelper.putBoolean(Category.PRE_MATCH, "Subsystems/Shooter", shooter != null);
    DashboardHelper.putBoolean(Category.PRE_MATCH, "Subsystems/Limelight", limelight != null);
    DashboardHelper.putBoolean(Category.PRE_MATCH, "Subsystems/TurretFeed", turretFeed != null);
    DashboardHelper.putBoolean(Category.PRE_MATCH, "Subsystems/Intake", intake != null);
    DashboardHelper.putBoolean(Category.PRE_MATCH, "Subsystems/LEDs", leds != null);
  }

  // Periodic Updates
  
  /** Updates vision pose and game state. Call from Robot.robotPeriodic(). */
  public void updateVisionPose() {
    // Always update calibration manager regardless of subsystem state
    calibrationManager.update();
    
    // Always send data to Pi (even when disabled, so Pi can stay connected)
    if (drivetrain != null && piShootingHelper != null) {
      piShootingHelper.update(
        drivetrain.getState().Pose,
        drivetrain.getState().Speeds,
        gameState.getTargetMode()
      );
    }
    
    if (limelight == null || drivetrain == null) return;
    
    limelight.setDrivetrainPose(drivetrain.getState().Pose);
    
    // Update game state with shuttle zone calculation - this enables auto-switching
    boolean inShuttleZone = calculateShuttleZone();
    gameState.update(inShuttleZone);
    
    // Log auto-switch status for debugging
    DashboardHelper.putBoolean(Category.DEBUG, "AutoShuttle/InZone", inShuttleZone);
    DashboardHelper.putBoolean(Category.DEBUG, "AutoShuttle/HasOverride", gameState.isShuttleModeManualOverride());
    DashboardHelper.putString(Category.TELEOP, "TargetMode", gameState.getTargetMode().name());
    
    if (limelight.hasVisionPose()) {
      double[] stdDevs = limelight.getVisionStdDevs();
      drivetrain.addVisionMeasurement(
          limelight.getRobotPose(),
          limelight.getPoseTimestamp(),
          VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]));
    }
    
    DashboardHelper.putBoolean(Category.TELEOP, "Drive/SlowMode", driverController.getRightTriggerAxis() > 0.3);
    DashboardHelper.putBoolean(Category.TELEOP, "Drive/RobotCentric", useRobotCentric);
  }
  
  private boolean calculateShuttleZone() {
    if (drivetrain == null) return false;
    
    double robotX = drivetrain.getState().Pose.getX();
    double boundaryX = DashboardHelper.getNumber(Category.SETTINGS, "Aim/AutoShuttleLineX", Constants.Field.AUTO_SHUTTLE_BOUNDARY_X);
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    
    // Blue alliance: past boundary = in shuttle zone (trench side)
    // Red alliance: before flipped boundary = in shuttle zone
    boolean inZone = (alliance == DriverStation.Alliance.Blue) 
        ? robotX > boundaryX 
        : robotX < (Constants.Field.FIELD_LENGTH_METERS - boundaryX);
    
    // Debug outputs to diagnose auto-switching
    DashboardHelper.putNumber(Category.DEBUG, "AutoShuttle/RobotX", robotX);
    DashboardHelper.putNumber(Category.DEBUG, "AutoShuttle/BoundaryX", 
        alliance == DriverStation.Alliance.Blue ? boundaryX : Constants.Field.FIELD_LENGTH_METERS - boundaryX);
    
    return inZone;
  }

  // Command Getters
  
  public Command getAimTurretCommand() {
    return (turret != null && limelight != null) 
        ? new AimTurretToPoseCommand(turret, limelight) 
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

  /** Turret gear ratio calibration - rotate turret 360 deg by hand while command runs. */
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
  public PiShootingHelper getPiShootingHelper() {
    return piShootingHelper;
  }

  /**
   * Explicitly stops all motor subsystems. Called from Robot.disabledInit()
   * as a belt-and-suspenders safety measure -- subsystem periodic() methods
   * also guard against disabled-state motor commands individually.
   */
  public void stopAllMotors() {
    if (shooter != null) shooter.stop();
    if (turretFeed != null) turretFeed.stop();
    if (intake != null) {
      intake.stopRollers();
      intake.retract();
    }
    // Turret: brake mode holds position passively, no explicit stop needed.
    // Drivetrain: WPILib command scheduler cancels drive commands on disable.
  }
}
