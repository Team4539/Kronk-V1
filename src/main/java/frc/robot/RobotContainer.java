package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
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
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.util.CANHealthMonitor;
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
    /** Whether defensive brake mode is active. Applies X-brake unless driver is commanding sticks. */
    private boolean defensiveMode = false;
    private final SendableChooser<Command> autoChooser;

    /** Whether vision has seeded the gyro heading at least once since boot. */
    private boolean hasSeededHeadingFromVision = false;
    
    /** Counter for shuttle zone logging to avoid spam */
    private int shuttleZoneLogCounter = 0;

    /** Last auto validation warning key to avoid spamming identical notifications */
    private String lastAutoWarning = "";
    private int autoValidationCounter = 0;

    /** Whether the drive team has pressed the Confirm Auto button */
    private boolean autoConfirmed = false;
    /** The auto name that was confirmed — if selection changes, un-confirm */
    private String confirmedAutoName = "";

    /** CAN bus health monitor for all devices */
    private final CANHealthMonitor canHealth;

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

        // 1b. CAN health monitor (needs all subsystem refs)
        canHealth = new CANHealthMonitor(drivetrain, shooter, trigger, intake, leds);

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
                    buildAutoShootCmd(20));
            NamedCommands.registerCommand("quickShoot",
                    buildAutoShootCmd(1.5));
            NamedCommands.registerCommand("TrainingShot",
                    buildAutoShootCmd(2.5));
        }
        if (shooter != null) {
            NamedCommands.registerCommand("spinUpShooter",
                    Commands.run(() -> shooter.setTargetRPM(
                            shootingCalc.getTargetRPM()), shooter));
            NamedCommands.registerCommand("stopShooter",
                    Commands.runOnce(() -> shooter.stop(), shooter));
        }
        // Intake commands
        if (intake != null) {
            NamedCommands.registerCommand("deployIntake",
                    new DeployIntakeCommand(intake));
            NamedCommands.registerCommand("retractIntake",
                    new RetractIntakeCommand(intake));
            NamedCommands.registerCommand("intakeJiggle",
                    new IntakeJiggleCommand(intake));
        }

        // Trigger: feed shot (trigger + intake jiggle, mimics normal shooting feed)
        if (trigger != null) {
            Command feedCmd = Commands.startEnd(
                    () -> trigger.setShoot(),
                    () -> trigger.stop(),
                    trigger);
            if (intake != null) {
                feedCmd = feedCmd.alongWith(new IntakeJiggleCommand(intake));
            }
            NamedCommands.registerCommand("feedShot", feedCmd.withTimeout(1.0));
        }

        // Stop everything (shooter + trigger + intake)
        NamedCommands.registerCommand("stopAll", Commands.runOnce(() -> {
            if (shooter != null) shooter.stop();
            if (trigger != null) trigger.stop();
            if (intake != null) intake.stopAndRetract();
        }));

        NamedCommands.registerCommand("enableShuttleMode",
                Commands.runOnce(() -> gameState.setShuttleMode(true)));
        NamedCommands.registerCommand("disableShuttleMode",
                Commands.runOnce(() -> gameState.setShuttleMode(false)));

        // Auto-aim: rotate robot to face the current target (hub or trench).
        // Uses robot-relative angleToTarget with simple P controller for rotation rate.
        if (drivetrain != null) {
            NamedCommands.registerCommand("aimAtPose",
                    Commands.runOnce(() -> resetAimController())
                    .andThen(drivetrain.applyRequest(() -> {
                        double errorDeg = shootingCalc.getAngleToTarget();
                        double omega = aimOmega(errorDeg);
                        return aimFieldCentric
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(omega);
                    }).until(() -> Math.abs(shootingCalc.getAngleToTarget())
                                    < Constants.Driver.AIM_TOLERANCE_DEG)
                     .withTimeout(1.5)));
        }

        // movingShoot: Shoot while PathPlanner keeps driving.
        // Hijacks only rotation (auto-aim) via drivetrain rotation override,
        // spools shooter, feeds trigger, then clears the override.
        // Does NOT require the drivetrain subsystem so PathPlanner keeps running.
        if (shooter != null && drivetrain != null) {
            NamedCommands.registerCommand("movingShoot",
                    Commands.runOnce(() -> {
                        resetAimController();
                        drivetrain.setRotationOverride(
                                () -> aimOmega(shootingCalc.getAngleToTarget()));
                    }).andThen(
                        // Spool shooter to target RPM and wait until ready + aimed
                        Commands.run(() -> shooter.setTargetRPM(shootingCalc.getTargetRPM()), shooter)
                                .until(() -> shooter.isReady()
                                        && Math.abs(shootingCalc.getAngleToTarget())
                                                < Constants.Driver.AIM_TOLERANCE_DEG)
                                .withTimeout(2.0)
                    ).andThen(
                        // Feed the shot
                        trigger != null
                                ? Commands.startEnd(() -> trigger.setShoot(), () -> trigger.stop(), trigger)
                                        .withTimeout(0.5)
                                : Commands.none()
                    ).finallyDo(() -> {
                        drivetrain.clearRotationOverride();
                        if (shooter != null) shooter.stop();
                    }));
        }

        NamedCommands.registerCommand("wait0.5", Commands.waitSeconds(0.5));
        NamedCommands.registerCommand("wait1.0", Commands.waitSeconds(1.0));
        NamedCommands.registerCommand("wait2.0", Commands.waitSeconds(2.0));

        // === COMBINED AUTO COMMANDS ===

        // spoolAndIntake: Spool shooter to target RPM + deploy intake at the same time.
        // Use while driving to a game piece — shooter is warm by the time you pick up.
        // Runs until interrupted (pair with path or timeout).
        if (shooter != null && intake != null) {
            NamedCommands.registerCommand("spoolAndIntake",
                    Commands.run(() -> shooter.setTargetRPM(shootingCalc.getTargetRPM()), shooter)
                            .alongWith(new DeployIntakeCommand(intake)));
        }

        // aimAndShoot: Rotate to face target, then auto-shoot once aimed + spun up.
        // Ends after feeding for 1 second (one ball cycle). Good for single-ball autos.
        if (shooter != null && vision != null && drivetrain != null) {
            NamedCommands.registerCommand("aimAndShoot",
                    buildAutoShootCmd(5.0));
        }

        // shootAndRetract: Feed a shot then retract intake. Clean end to a scoring cycle.
        if (trigger != null) {
            Command shootRetract = Commands.startEnd(
                    () -> trigger.setShoot(),
                    () -> trigger.stop(),
                    trigger);
            if (intake != null) {
                shootRetract = shootRetract.alongWith(new IntakeJiggleCommand(intake))
                        .withTimeout(1.0)
                        .andThen(Commands.runOnce(() -> intake.stopAndRetract()));
            } else {
                shootRetract = shootRetract.withTimeout(1.0);
            }
            NamedCommands.registerCommand("shootAndRetract", shootRetract);
        }

        // forceShootOn / forceShootOff: Bypass alliance timing during auto.
        // Use at start of auto to guarantee shooting works regardless of FMS timing.
        NamedCommands.registerCommand("forceShootOn",
                Commands.runOnce(() -> gameState.setForceShootEnabled(true)));
        NamedCommands.registerCommand("forceShootOff",
                Commands.runOnce(() -> gameState.setForceShootEnabled(false)));

        // waitForShooterReady: Blocks until shooter is within RPM tolerance.
        // Insert between spinUpShooter and feedShot to avoid shooting before ready.
        if (shooter != null) {
            NamedCommands.registerCommand("waitForShooterReady",
                    Commands.waitUntil(() -> shooter.isReady()).withTimeout(2.0));
        }

        // waitForAimed: Blocks until robot is aimed at target (within tolerance).
        // Use after aimAtPose if you want to gate on aim before feeding.
        NamedCommands.registerCommand("waitForAimed",
                Commands.waitUntil(() -> Math.abs(shootingCalc.getAngleToTarget())
                        < Constants.Driver.AIM_TOLERANCE_DEG).withTimeout(2.0));

        // spoolToHub / spoolToTrench: Pre-spool shooter while driving toward target.
        // Runs until interrupted — use alongside a path.
        if (shooter != null) {
            NamedCommands.registerCommand("spoolToHub",
                    Commands.run(() -> shooter.setTargetRPM(shootingCalc.getTargetRPM()), shooter));
            NamedCommands.registerCommand("spoolToTrench",
                    Commands.runOnce(() -> gameState.setShuttleMode(true))
                            .andThen(Commands.run(() -> shooter.setTargetRPM(
                                    shootingCalc.getTargetRPM()), shooter)));
        }

        // intakeAndDrive: Deploy intake with rollers spinning — use alongside a path
        // to pick up game pieces while driving. Runs until interrupted.
        if (intake != null) {
            NamedCommands.registerCommand("intakeAndDrive",
                    new DeployIntakeCommand(intake));
        }

        // fullCycle: Aim + shoot + retract in one command. The "do everything" button.
        // Aims the robot, spools shooter, feeds when ready, then retracts intake.
        if (shooter != null && vision != null && drivetrain != null) {
            NamedCommands.registerCommand("fullCycle",
                    buildAutoShootCmd(4.0)
                            .andThen(Commands.runOnce(() -> {
                                if (intake != null) intake.stopAndRetract();
                            })));
        }
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
     * A:           Defensive brake mode toggle (X-brake unless driving)
     * B:           Force shoot toggle (with LED feedback)
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
                if (leds != null) {
                    // Show GREEN when shooter is at speed AND robot is aimed (ready to fire!)
                    // Show ORANGE breathing while still spooling up or aiming
                    boolean aimed = Math.abs(shootingCalc.getAngleToTarget()) < Constants.Driver.AIM_TOLERANCE_DEG;
                    if (shooter.isReady() && aimed) {
                        leds.setAction(LEDSubsystem.ActionState.SHOOTING);
                    } else {
                        leds.setAction(LEDSubsystem.ActionState.SPOOLING);
                    }
                }
            }, shooter).finallyDo(() -> {
                shooter.stop();
                if (leds != null) leds.clearAction();
            });

            
            
                driver.leftTrigger(0.5).whileTrue(spoolCmd);
            
        }

        // LB: Auto-shoot (hold) — also jiggles intake to keep balls feeding
        // Auto-aims robot toward target via camera while shooting.
        if (shooter != null && vision != null) {
            Command shootCmd = new AutoShootCommand(shooter, vision, leds, trigger, drivetrain);
            
            Command fullCmd = Commands.runOnce(() -> resetAimController()).andThen(shootCmd);
            if (intake != null) {
                fullCmd = fullCmd.alongWith(new IntakeJiggleCommand(intake));
            }
            // Auto-aim + shoot-on-the-fly: latch the robot's movement direction
            // when LB is first pressed, then drive at a fixed speed in that direction.
            // This gives predictable velocity for shoot-on-the-move compensation.
            if (drivetrain != null) {
                Command latchCmd = Commands.runOnce(() -> {
                    var speeds = drivetrain.getState().Speeds;
                    double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                    if (speed > 0.1) {
                        latchedDriveAngle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
                        shootOnFlyLatched = false;
                    } else {
                        shootOnFlyLatched = false;
                    }
                });
                Command aimCmd = drivetrain.applyRequest(() -> {
                    // Brake lock while shooting if ready + aimed + not trying to drive
                    boolean aimed = Math.abs(shootingCalc.getAngleToTarget()) < Constants.Driver.AIM_TOLERANCE_DEG;
                    if (shooter.isReady() && aimed && !isDriverCommanding() && !shootOnFlyLatched) {
                        return brake;
                    }
                    double vx, vy;
                    if (shootOnFlyLatched) {
                        // Drive at fixed speed in the latched direction
                        double spd = Constants.Shooter.SHOOT_ON_FLY_SPEED_MPS;
                        vx = Math.cos(latchedDriveAngle) * spd;
                        vy = Math.sin(latchedDriveAngle) * spd;
                    } else {
                        // Not moving when LB pressed — let driver control translation
                        double slow = driver.getRightTriggerAxis() > 0.3
                                ? Constants.Driver.SLOW_MODE_MULTIPLIER : 1.0;
                        vx = -driver.getLeftY() * Constants.Driver.MAX_DRIVE_SPEED_MPS * slow;
                        vy = -driver.getLeftX() * Constants.Driver.MAX_DRIVE_SPEED_MPS * slow;
                    }
                    double omega = aimOmega(shootingCalc.getAngleToTarget());
                    return aimFieldCentric
                            .withVelocityX(vx)
                            .withVelocityY(vy)
                            .withRotationalRate(omega);
                });
                fullCmd = fullCmd.alongWith(latchCmd.andThen(aimCmd).finallyDo(() -> shootOnFlyLatched = false));
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

            // POV Left: Reverse intake (deploy + spit out, hold/release)
            driver.a().whileTrue(
                    Commands.startEnd(
                            () -> {
                                intake.deploy();
                                intake.setRollerSpeed(Constants.Intake.OUTTAKE_SPEED_RPM);
                            },
                            () -> {
                                intake.stopAndRetract();
                            },
                            intake)
                            .alongWith(ledAction(LEDSubsystem.ActionState.REVERSE_INTAKE))
                            .finallyDo(() -> {
                                if (leds != null) leds.clearAction();
                            }));
        }

        // A: Defensive brake mode toggle (X-brake unless driver drives)
        driver.povDownLeft().onTrue(Commands.runOnce(() -> {
            defensiveMode = !defensiveMode;
            if (leds != null) {
                if (defensiveMode) {
                    leds.setMode(LEDSubsystem.ModeState.DEFENSIVE);
                } else {
                    leds.clearMode();
                }
            }
            notify(defensiveMode ? "Defensive Mode ON" : "Defensive Mode OFF");
        }));

        // B: Force shoot toggle (with LED feedback)
        driver.b().onTrue(Commands.runOnce(() -> {
            boolean on = !gameState.isForceShootEnabled();
            gameState.setForceShootEnabled(on);
            if (leds != null) {
                if (on) {
                    leds.setMode(LEDSubsystem.ModeState.FORCE_SHOOT);
                } else {
                    leds.clearMode();
                }
            }
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
                // SAFETY: Do not drive while disabled; send brake request
                if (DriverStation.isDisabled()) {
                    return brake;
                }

                // DEFENSIVE MODE: X-brake unless driver is commanding sticks
                if (defensiveMode && !isDriverCommanding()
                        && Math.abs(driver.getRightX()) < Constants.Driver.STICK_DEADBAND) {
                    return brake;
                }

                double slow = driver.getRightTriggerAxis() > 0.3
                        ? Constants.Driver.SLOW_MODE_MULTIPLIER : 1.0;
                double vx = -driver.getLeftY()  * Constants.Driver.MAX_DRIVE_SPEED_MPS  * slow;
                double vy = -driver.getLeftX()  * Constants.Driver.MAX_DRIVE_SPEED_MPS  * slow;

                // On-the-fly auto-aim: LT held → auto-rotate toward target
                boolean autoAim = driver.getLeftTriggerAxis() > 0.5;
                if (autoAim && !wasAutoAiming) {
                    resetAimController();
                }
                wasAutoAiming = autoAim;
                if (autoAim) {
                    // BRAKE LOCK: When shooter is spun up and robot is aimed at the target,
                    // lock wheels in X-pattern to resist defense — unless driver is driving.
                    boolean aimed = Math.abs(shootingCalc.getAngleToTarget()) < Constants.Driver.AIM_TOLERANCE_DEG;
                    if (shooter != null && shooter.isReady() && aimed && !isDriverCommanding()) {
                        return brake;
                    }
                    double omega = aimOmega(shootingCalc.getAngleToTarget());
                    // Use aimFieldCentric (no rotational deadband) for precise auto-aim
                    return aimFieldCentric.withVelocityX(vx).withVelocityY(vy)
                            .withRotationalRate(omega);
                }

                double vr = -driver.getRightX() * Constants.Driver.MAX_ANGULAR_SPEED_RAD * slow
                          * Constants.Driver.AIM_DIRECTION;
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
                // SAFETY: Do not spin shooter while disabled
                if (DriverStation.isDisabled()) {
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

        // Auto confirmation button (momentary — code resets to false after reading)
        SmartDashboard.putBoolean("Auto/Confirm", false);
        SmartDashboard.putBoolean("Auto/Confirmed", false);
        SmartDashboard.putString("Auto/SelectedName", "");

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

        updateRumble();
        canHealth.update();

        calibration.update();

        // Update shooting calculator with current pose
        if (drivetrain != null) {
            // Cache state once per cycle for consistency and to avoid redundant CAN reads
            var driveState = drivetrain.getState();

            // In calibration mode, pass the absolute RPM slider value so
            // ShootingCalculator uses it directly (bypasses interpolation table).
            // In normal mode, pass the RPM offset added on top of interpolated RPM.
            double rpmParam = shootingCalc.isCalibrationMode()
                    ? calibration.getShooterRPM()
                    : calibration.getRPMOffset();
            shootingCalc.update(
                    driveState.Pose,
                    driveState.Speeds,
                    gameState.getTargetMode(),
                    rpmParam);
            calibration.setCurrentDistance(shootingCalc.getDistance());
        }

        if (vision == null || drivetrain == null) return;

        // Cache state for vision pipeline (may differ from above if drivetrain was null)
        var driveStateVision = drivetrain.getState();

        // Feed drivetrain pose to vision for fallback
        vision.setDrivetrainPose(driveStateVision.Pose);

        // Feed gyro heading to vision pose estimator for better single-tag estimates.
        double yawDegrees = driveStateVision.Pose.getRotation().getDegrees();
        double yawRate = Math.toDegrees(driveStateVision.Speeds.omegaRadiansPerSecond);
        vision.setRobotOrientation(yawDegrees, yawRate, 0, 0, 0, 0);

        // Auto-shuttle zone check — only after vision has seeded heading,
        // otherwise robot pose is (0,0) and zone detection is wrong
        if (hasSeededHeadingFromVision) {
            gameState.update(isInShuttleZone());

            // Zone-based LED modes — only during teleop, don't stomp manual modes
            if (leds != null && DriverStation.isTeleopEnabled()) {
                boolean inAllianceZone = isInAllianceZone();
                boolean inOpposingZone = isInShuttleZone();

                // Opposing zone → auto-defensive LED
                if (inOpposingZone && !wasInOpposingZone) {
                    if (leds.getMode() == LEDSubsystem.ModeState.NONE
                            || leds.getMode() == LEDSubsystem.ModeState.ALLIANCE_ZONE) {
                        leds.setMode(LEDSubsystem.ModeState.OPPOSING_ZONE);
                    }
                } else if (!inOpposingZone && wasInOpposingZone) {
                    if (leds.getMode() == LEDSubsystem.ModeState.OPPOSING_ZONE) {
                        leds.clearMode();
                    }
                }
                wasInOpposingZone = inOpposingZone;

                // Alliance zone → alliance color pulse (lower priority than opposing)
                if (inAllianceZone && !wasInAllianceZone) {
                    if (leds.getMode() == LEDSubsystem.ModeState.NONE) {
                        leds.setMode(LEDSubsystem.ModeState.ALLIANCE_ZONE);
                    }
                } else if (!inAllianceZone && wasInAllianceZone) {
                    if (leds.getMode() == LEDSubsystem.ModeState.ALLIANCE_ZONE) {
                        leds.clearMode();
                    }
                }
                wasInAllianceZone = inAllianceZone;
            }
        } else {
            gameState.update();
        }

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
                // Single-tag: trust X/Y, use gyro heading for rotation
                // Hard-reset pose so we snap to the correct field position immediately
                edu.wpi.first.math.geometry.Pose2d corrected = new edu.wpi.first.math.geometry.Pose2d(
                        seedPose.getX(), seedPose.getY(),
                        drivetrain.getState().Pose.getRotation());
                drivetrain.resetPose(corrected);
                hasSeededHeadingFromVision = true;
                vision.notifyHeadingSeeded();
                System.out.println("[RobotContainer] Vision pose initialized (SingleTag, gyro heading): "
                        + String.format("(%.2f, %.2f) @ %.1f\u00b0",
                                seedPose.getX(), seedPose.getY(),
                                corrected.getRotation().getDegrees()));
                Elastic.sendNotification(
                        new Elastic.Notification(NotificationLevel.INFO,
                                "Vision Pose Initialized",
                                String.format("SingleTag ID %d: (%.1f, %.1f) - heading from gyro",
                                        vision.getTargetId(),
                                        seedPose.getX(), seedPose.getY())));
            }
        }

        // --- Fuse front camera's pose into drivetrain odometry ---
        if (vision.hasFrontPose()) {
            fuseCamera(vision.getFrontPose(), vision.getFrontTimestamp(),
                       vision.isFrontMultiTag(), vision.getFrontTargetCount());
        }

        // --- Stall/jam LED warning — any motor reversing to clear a jam ---
        if (leds != null && DriverStation.isEnabled()) {
            boolean anyStall = (shooter != null && shooter.isStalled())
                            || (trigger != null && trigger.isStalled());
            if (anyStall) {
                leds.setAction(LEDSubsystem.ActionState.STALL_JAM);
            } else if (leds.getAction() == LEDSubsystem.ActionState.STALL_JAM) {
                leds.clearAction();
            }
        }

        // --- Vision-lost LED warning (teleop only, low priority) ---
        if (leds != null && DriverStation.isTeleopEnabled() && hasSeededHeadingFromVision) {
            if (!vision.hasVisionPose()) {
                if (leds.getAction() == LEDSubsystem.ActionState.IDLE) {
                    leds.setAction(LEDSubsystem.ActionState.VISION_LOST);
                }
            } else if (leds.getAction() == LEDSubsystem.ActionState.VISION_LOST) {
                leds.clearAction();
            }
        }
    }

    private boolean isInShuttleZone() {
        if (drivetrain == null) {
            System.out.println("[RobotContainer] WARNING: Drivetrain is null, cannot check shuttle zone");
            return false;
        }
        
        double x = drivetrain.getState().Pose.getX();
        double boundary = Constants.Field.AUTO_SHUTTLE_BOUNDARY_X;
        boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Blue;
        
        boolean inZone = isBlue ? x > boundary : x < (Constants.Field.FIELD_LENGTH_METERS - boundary);
        
        // Log shuttle zone detection every 50 cycles (1 second) to avoid spam
        shuttleZoneLogCounter++;
        if (shuttleZoneLogCounter >= 50) {
            shuttleZoneLogCounter = 0;
            System.out.println("[RobotContainer] Shuttle Zone Check: " +
                             "x=" + String.format("%.2f", x) +
                             ", boundary=" + String.format("%.2f", boundary) +
                             ", alliance=" + (isBlue ? "BLUE" : "RED") +
                             ", inZone=" + inZone +
                             ", autoEnabled=" + Constants.Field.AUTO_SHUTTLE_ENABLED +
                             ", shuttleMode=" + gameState.isShuttleMode() +
                             ", manualOverride=" + gameState.isShuttleModeManualOverride());
        }
        
        return inZone;
    }

    private boolean wasInAllianceZone = false;
    private boolean wasInOpposingZone = false;

    private boolean isInAllianceZone() {
        if (drivetrain == null) return false;
        double x = drivetrain.getState().Pose.getX();
        double boundary = Constants.Field.ALLIANCE_ZONE_BOUNDARY_X;
        boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Blue;
        // Alliance zone: blue = low X (near blue wall), red = high X (near red wall)
        return isBlue ? x < boundary : x > (Constants.Field.FIELD_LENGTH_METERS - boundary);
    }

    // =========================================================================
    // PUBLIC GETTERS (used by Robot.java)
    // =========================================================================

    public Command getAutonomousCommand() { return autoChooser.getSelected(); }
    public LEDSubsystem getLEDs() { return leds; }

    /** Clear defensive mode and persistent LED modes on mode transitions. */
    public void clearDefensiveMode() {
        defensiveMode = false;
        if (leds != null) {
            leds.clearAction();
            leds.clearMode();
        }
    }

    /** Stop all motors — called from Robot.disabledInit() for safety. */
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

    /**
     * Returns true when the driver is commanding significant translation (left stick).
     * Used to release swerve brake lock during shooting — if the driver pushes a stick
     * they can escape defense even while the brake-on-aim feature is active.
     */
    private boolean isDriverCommanding() {
        double deadband = Constants.Driver.STICK_DEADBAND;
        return Math.abs(driver.getLeftY()) > deadband
            || Math.abs(driver.getLeftX()) > deadband;
    }

    /** Previous aim error in radians — used for D term to damp oscillation */
    private double previousAimErrorRad = 0.0;
    /** Track whether auto-aim was active last cycle to reset D-term on entry */
    private boolean wasAutoAiming = false;

    /** Latched field-relative direction (radians) for shoot-on-the-fly */
    private double latchedDriveAngle = 0.0;
    /** Whether the shoot-on-the-fly direction has been latched this press */
    private boolean shootOnFlyLatched = false;

    /** Reset the PD controller state to avoid D-term spikes when switching aim commands */
    public void resetAimController() {
        previousAimErrorRad = 0.0;
    }

    /** Builds an auto-shoot named command that rotates to aim while shooting. */
    private Command buildAutoShootCmd(double timeoutSeconds) {
        Command shootCmd = new AutoShootCommand(shooter, vision, leds, trigger, drivetrain);
        Command fullCmd = Commands.runOnce(() -> resetAimController()).andThen(shootCmd);
        if (drivetrain != null) {
            Command aimCmd = drivetrain.applyRequest(() -> {
                double omega = aimOmega(shootingCalc.getAngleToTarget());
                return aimFieldCentric
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(omega);
            });
            fullCmd = fullCmd.alongWith(aimCmd);
        }
        if (intake != null) {
            fullCmd = fullCmd.alongWith(new IntakeJiggleCommand(intake));
        }
        return fullCmd.withTimeout(timeoutSeconds);
    }

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
     * Uses AIM_DIRECTION for manual rotation sign. The auto-aim P/D output
     * is negated to drive the error toward zero (robot rotates toward target).
     */
    private double aimOmega(double errorDeg) {
        // Guard against NaN/Infinity from bad vision data — safe fallback to no rotation
        if (Double.isNaN(errorDeg) || Double.isInfinite(errorDeg)) {
            previousAimErrorRad = 0.0;
            return 0.0;
        }

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

        double omega = (pTerm + dTerm);

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

    // =========================================================================
    // RUMBLE FEEDBACK
    // =========================================================================

    /**
     * Drives Xbox controller rumble based on game state:
     *   LEFT rumble  = shooting readiness (aiming + shooter at RPM)
     *   RIGHT rumble = shift timing warnings (get ready / go now)
     *   BOTH         = shift is NOW — shoot immediately!
     */
    private void updateRumble() {
        if (DriverStation.isDisabled()) {
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
            return;
        }

        double leftRumble = 0;
        double rightRumble = 0;

        // --- Shift change warnings (right rumble → both at peak) ---
        if (gameState.isShootNowWarning()) {
            // SHOOT NOW — heavy both-sides rumble
            leftRumble = 1.0;
            rightRumble = 1.0;
        } else if (gameState.isSpoolWarning()) {
            // Shift coming in 1-5 sec — moderate right rumble
            rightRumble = 0.4;
        } else if (gameState.isHeadBackWarning()) {
            // 5-10 sec out — light nudge
            rightRumble = 0.15;
        }

        // --- Ready to fire (left rumble, only when driver is aiming AND can score) ---
        boolean aiming = driver.getLeftTriggerAxis() > 0.5
                      || driver.leftBumper().getAsBoolean();
        boolean aimed = Math.abs(shootingCalc.getAngleToTarget())
                      < Constants.Driver.AIM_TOLERANCE_DEG;
        boolean shooterReady = shooter != null && shooter.isReady();
        boolean canScore = gameState.canShoot();

        if (aiming && aimed && shooterReady && canScore) {
            // Locked on + spun up + allowed to score → FIRE!
            leftRumble = Math.max(leftRumble, 0.6);
        } else if (aiming && shooterReady && canScore) {
            // Spun up but still rotating → almost there
            leftRumble = Math.max(leftRumble, 0.15);
        }

        driver.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, leftRumble);
        driver.getHID().setRumble(GenericHID.RumbleType.kRightRumble, rightRumble);
    }

    // =========================================================================
    // AUTO SELECTOR VALIDATION (call from disabledPeriodic)
    // =========================================================================

    /**
     * Auto selection validation with confirmation button.
     *
     * The drive team MUST press the "Confirm Auto" button on the dashboard
     * before the warning clears. This prevents stale auto selections from
     * Elastic persisting across reboots.
     *
     * - Any auto (including None/InstantCommand) can be confirmed
     * - If the selection changes after confirmation, it un-confirms
     * - "Auto/Confirm" is a momentary button — code resets it to false immediately
     */
    public void validateAutoSelection() {
        // --- Read current selection ---
        Command selected = autoChooser.getSelected();
        String autoName = selected != null ? selected.getName() : "None";
        if (autoName.isEmpty() || autoName.equals("InstantCommand")) autoName = "None";
        SmartDashboard.putString("Auto/SelectedName", autoName);

        // --- Momentary confirm button: read, act, reset ---
        if (SmartDashboard.getBoolean("Auto/Confirm", false)) {
            SmartDashboard.putBoolean("Auto/Confirm", false); // Reset immediately (momentary)
            autoConfirmed = true;
            confirmedAutoName = autoName;
            System.out.println("[Auto] Confirmed: " + autoName);
            Elastic.sendNotification(new Elastic.Notification()
                    .withLevel(NotificationLevel.INFO)
                    .withTitle("Auto Confirmed")
                    .withDescription(autoName)
                    .withDisplaySeconds(2.0));
        }

        // --- Un-confirm if selection changed ---
        if (autoConfirmed && !autoName.equals(confirmedAutoName)) {
            autoConfirmed = false;
            System.out.println("[Auto] Selection changed (" + confirmedAutoName + " -> " + autoName + ") — re-confirm required");
        }

        SmartDashboard.putBoolean("Auto/Confirmed", autoConfirmed);
        SmartDashboard.putBoolean("Auto/Valid", autoConfirmed);

        // --- Warning logic (throttled to every ~5 seconds) ---
        autoValidationCounter++;
        if (autoValidationCounter < 250) return;
        autoValidationCounter = 0;

        if (!autoConfirmed) {
            if (!lastAutoWarning.equals("UNCONFIRMED")) {
                lastAutoWarning = "UNCONFIRMED";
                SmartDashboard.putString("Auto/Warning", "CONFIRM AUTO!");
            }
            Elastic.sendNotification(new Elastic.Notification()
                    .withLevel(NotificationLevel.ERROR)
                    .withTitle("AUTO NOT CONFIRMED")
                    .withDescription("Selected: " + autoName + " — press Confirm!")
                    .withWidth(500)
                    .withHeight(300)
                    .withDisplaySeconds(6));
            if (leds != null) leds.setAction(LEDSubsystem.ActionState.NO_AUTO);
        } else {
            if (!lastAutoWarning.isEmpty()) {
                lastAutoWarning = "";
                SmartDashboard.putString("Auto/Warning", "");
                if (leds != null && leds.getAction() == LEDSubsystem.ActionState.NO_AUTO) {
                    leds.clearAction();
                }
            }
        }
    }
}
