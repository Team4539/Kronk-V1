package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.GameStateManager.TargetMode;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.ActionState;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretFeedSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.ShootingCalculator;

/**
 * Fully automated shooting command. Handles turret aiming, shooter spin-up,
 * and ball feeding in one seamless operation.
 * 
 * Alliance-aware: only feeds balls when our scoring window is active
 * (or during green-light pre-shift / force-shoot override).
 * 
 * All turret/shooter calculations come from ShootingCalculator, which is
 * updated every cycle by RobotContainer.updateVisionPose(). This command
 * reads those results and controls the subsystems accordingly.
 */
public class AutoShootCommand extends Command {
    
    // SUBSYSTEMS & STATE MANAGER
    
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    @SuppressWarnings("unused")
    private final CommandSwerveDrivetrain drivetrain; // Reserved for future use (e.g., auto-stop while shooting)
    private final GameStateManager gameState;
    private final LEDSubsystem leds; // Optional - can be null
    private final TurretFeedSubsystem turretFeed; // Optional - can be null
    private final ShootingCalculator shootingCalc;
    
    // STATE TRACKING
    
    /** Whether turret is within tolerance of target angle */
    private boolean turretOnTarget = false;
    
    /** Whether shooter has spun up sufficiently */
    private boolean shooterReady = false;
    
    /** Whether our alliance is currently allowed to score */
    private boolean allianceActive = false;
    
    /** Distance to current target (hub or trench) */
    private double distanceToTarget = 0.0;
    
    /** Calculated turret angle to target */
    private double turretTargetAngle = 0.0;
    
    /** Current target mode from game state manager */
    private TargetMode currentTargetMode = TargetMode.DISABLED;
    
    /** Whether currently moving fast enough to need lead compensation */
    private boolean isMoving = false;
    
    /** Tracks notification state to avoid spamming */
    private boolean hasNotifiedReady = false;
    private boolean wasAllianceActive = false;
    private boolean hasNotifiedHeadBack = false;
    private boolean hasNotifiedGreenLight = false;
    
    /** Tracks whether we were firing last cycle (for snapshot rising edge detection) */
    private boolean wasFiring = false;
    
    // CONSTRUCTOR
    
    /**
     * Creates the auto shoot command (turret + shooter only, no feed motor).
     * 
     * @param turret The turret subsystem for aiming
     * @param shooter The shooter subsystem for launching
     * @param vision The vision subsystem for pose estimation
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter, VisionSubsystem vision) {
        this(turret, shooter, vision, null, null, null);
    }
    
    /**
     * Creates the auto shoot command with LED feedback but no feed motor.
     * 
     * @param turret The turret subsystem for aiming
     * @param shooter The shooter subsystem for launching
     * @param vision The vision subsystem for pose estimation
     * @param leds The LED subsystem for visual feedback (optional, can be null)
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter, VisionSubsystem vision, LEDSubsystem leds) {
        this(turret, shooter, vision, leds, null, null);
    }
    
    /**
     * Creates the auto shoot command with LED feedback and feed motor control.
     * When turretFeed is provided, balls are automatically fed into the shooter when ready.
     * 
     * @param turret The turret subsystem for aiming
     * @param shooter The shooter subsystem for launching
     * @param vision The vision subsystem for pose estimation
     * @param leds The LED subsystem for visual feedback (optional, can be null)
     * @param turretFeed The turret feed subsystem for feeding balls (optional, can be null)
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter, VisionSubsystem vision, 
                           LEDSubsystem leds, TurretFeedSubsystem turretFeed) {
        this(turret, shooter, vision, leds, turretFeed, null);
    }
    
    /**
     * Creates the full auto shoot command with all subsystems.
     * Lead angle compensation is handled by ShootingCalculator using drivetrain
     * velocity data provided by RobotContainer.updateVisionPose().
     * 
     * @param turret The turret subsystem for aiming
     * @param shooter The shooter subsystem for launching
     * @param vision The vision subsystem for pose estimation
     * @param leds The LED subsystem for visual feedback (optional, can be null)
     * @param turretFeed The turret feed subsystem for feeding balls (optional, can be null)
     * @param drivetrain The swerve drivetrain (optional, reserved for future use)
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter, VisionSubsystem vision, 
                           LEDSubsystem leds, TurretFeedSubsystem turretFeed,
                           CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.leds = leds;
        this.turretFeed = turretFeed;
        this.gameState = GameStateManager.getInstance();
        this.shootingCalc = ShootingCalculator.getInstance();
        
        // Requires turret and shooter, plus feed motor if provided
        addRequirements(turret, shooter);
        if (turretFeed != null) {
            addRequirements(turretFeed);
        }
    }

    // COMMAND LIFECYCLE

    @Override
    public void initialize() {
        SmartDashboard.putString("AutoShoot/Status", "Initializing");
        turretOnTarget = false;
        shooterReady = false;
        hasNotifiedReady = false;
        wasAllianceActive = false;
        hasNotifiedHeadBack = false;
        hasNotifiedGreenLight = false;
        wasFiring = false;
    }

    @Override
    public void execute() {
        // ----- Check if auto-aiming is disabled in practice/test mode -----
        if (!gameState.isAutoAimEnabled()) {
            SmartDashboard.putString("AutoShoot/Status", "AUTO-AIM DISABLED");
            shooter.stop();
            updateLEDState();
            publishTelemetry();
            return;
        }
        
        // ----- Update state from game manager -----
        currentTargetMode = gameState.getTargetMode();
        allianceActive = gameState.isOurAllianceActive();
        
        // ----- Check for valid pose -----
        if (!vision.hasPoseEstimate()) {
            SmartDashboard.putString("AutoShoot/Status", "No Pose - Cannot Aim");
            shooter.stop();
            stopBallHandling();
            updateLEDState();
            publishTelemetry();
            return;
        }
        
        // ----- Handle disabled mode (alliance inactive) -----
        // Green light pre-shift (3 sec before our shift) OR Force Shoot overrides disabled
        if (currentTargetMode == TargetMode.DISABLED && !gameState.isGreenLightPreShift() && !gameState.isForceShootEnabled()) {
            SmartDashboard.putString("AutoShoot/Status", "ALLIANCE INACTIVE - Waiting");
            // If we're in head-back warning (5 sec), still aim but don't shoot
            if (gameState.isHeadBackWarning()) {
                SmartDashboard.putString("AutoShoot/Status", "HEAD BACK - Shift in " + 
                    String.format("%.1f", gameState.getSecondsUntilOurNextShift()) + "s");
                aimAtTarget();
                setShooterWithTraining(); // Pre-spool during head-back
            } else {
                shooter.stop();
                stopBallHandling();
                // Still aim so we're ready when window opens
                aimAtTarget();
            }
            updateLEDState();
            publishTelemetry();
            return;
        }
        
        // ----- Aim at the appropriate target (with training corrections) -----
        aimAtTarget();
        
        // ----- Set shooter power with training corrections -----
        setShooterWithTraining();
        
        // ----- Check turret on-target status -----
        double turretError = Math.abs(turretTargetAngle - turret.getCurrentAngle());
        turretOnTarget = turretError < Constants.Turret.ON_TARGET_TOLERANCE_DEG;
        
        // ----- Check shooter ready status -----
        shooterReady = shooter.isReady();
        
        // ----- Control turret feed motor -----
        // Only feed balls when turret is aimed, shooter is spun up, and alliance is active
        boolean firing = isReadyToFire();
        if (firing) {
            // Feed balls into the shooter
            if (turretFeed != null) {
                turretFeed.setShoot();
            }
        } else {
            // Not ready - keep feed in idle mode
            idleBallHandling();
        }
        wasFiring = firing;
        
        // ----- Update LED action state -----
        updateLEDState();
        
        // ----- Send Elastic notifications for key events -----
        updateNotifications();
        
        // ----- Update status display -----
        String status = determineStatus();
        SmartDashboard.putString("AutoShoot/Status", status);
        
        publishTelemetry();
    }
    
    /**
     * Sends Elastic notifications when important events happen.
     * Keeps track of state to avoid spamming the same notification.
     */
    private void updateNotifications() {
        // Notify when we become ready to fire (only once per ready cycle)
        if (isReadyToFire() && !hasNotifiedReady) {
            hasNotifiedReady = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle("READY TO FIRE!")
                .withDescription(String.format("Distance: %.1fm | %s", distanceToTarget, 
                    currentTargetMode == TargetMode.TRENCH ? "Trench" : "Hub"))
                .withDisplaySeconds(1.5));
        } else if (!isReadyToFire()) {
            hasNotifiedReady = false; // Reset so we can notify again next time
        }
        
        // Notify when alliance window changes (important for timing!)
        if (allianceActive && !wasAllianceActive) {
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle("Alliance Window OPEN")
                .withDescription("Scoring is now allowed!")
                .withDisplaySeconds(2.0));
            hasNotifiedHeadBack = false;
            hasNotifiedGreenLight = false;
        } else if (!allianceActive && wasAllianceActive && !gameState.isForceShootEnabled()) {
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle("Alliance Window CLOSED")
                .withDescription("Waiting for next window...")
                .withDisplaySeconds(2.0));
        }
        wasAllianceActive = allianceActive;
        
        // Notify head-back warning (5 sec before our shift)
        if (gameState.isHeadBackWarning() && !hasNotifiedHeadBack) {
            hasNotifiedHeadBack = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle("<- HEAD BACK!")
                .withDescription(String.format("Our shift starts in %.0fs - get in position!", 
                    gameState.getSecondsUntilOurNextShift()))
                .withDisplaySeconds(2.0));
        } else if (!gameState.isHeadBackWarning()) {
            hasNotifiedHeadBack = false;
        }
        
        // Notify green light (3 sec before our shift - can start shooting!)
        if (gameState.isGreenLightPreShift() && !hasNotifiedGreenLight) {
            hasNotifiedGreenLight = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle("GREEN LIGHT!")
                .withDescription("Cleared to shoot! Shift starts in " + 
                    String.format("%.0fs", gameState.getSecondsUntilOurNextShift()))
                .withDisplaySeconds(2.0));
        } else if (!gameState.isGreenLightPreShift()) {
            hasNotifiedGreenLight = false;
        }
    }
    
    /**
     * Updates LED action state based on current shooting status.
     * FIRING = feed motor actively running (highest priority)
     * SHOOTING = ready to fire but not yet feeding
     * AIMING/SPOOLING = still getting ready
     */
    private void updateLEDState() {
        if (leds == null) return;
        
        // Determine priority action state
        if (wasFiring) {
            // Balls are actively being launched! Most dramatic animation.
            leds.setAction(ActionState.FIRING);
        } else if (turretOnTarget && shooterReady && (allianceActive || gameState.isGreenLightPreShift() || gameState.isForceShootEnabled())) {
            // Ready to shoot but not yet feeding -- tell driver to hold/fire!
            leds.setAction(ActionState.SHOOTING);
        } else if (!turretOnTarget && !shooterReady) {
            // Both aiming and spooling - prioritize aiming
            leds.setAction(ActionState.AIMING);
        } else if (!turretOnTarget) {
            // Only aiming
            leds.setAction(ActionState.AIMING);
        } else if (!shooterReady) {
            // Only spooling up
            leds.setAction(ActionState.SPOOLING);
        } else {
            // Shouldn't get here, but default to aiming
            leds.setAction(ActionState.AIMING);
        }
    }
    
    // FEED MOTOR CONTROL
    
    /**
     * Sets the turret feed motor to idle speed.
     */
    private void idleBallHandling() {
        if (turretFeed != null) {
            turretFeed.setIdle();
        }
    }
    
    /**
     * Stops the turret feed motor completely.
     */
    private void stopBallHandling() {
        if (turretFeed != null) {
            turretFeed.stop();
        }
    }
    
    // AIMING LOGIC
    
    /**
     * Aims at the current target using ShootingCalculator results.
     * 
     * NOTE: shootingCalc.update() is called from RobotContainer.updateVisionPose() every cycle,
     * so we only READ the solution here.
     * 
     * All turret angle offsets (global, position-based, live calibration) are already applied
     * inside ShootingCalculator.update(). Do NOT add additional offsets here or you will
     * double-correct, making calibration impossible to reproduce.
     */
    private void aimAtTarget() {
        // Read ShootingCalculator results -- already updated by RobotContainer
        turretTargetAngle = shootingCalc.getTurretAngle();
        distanceToTarget = shootingCalc.getDistance();
        isMoving = shootingCalc.isMoving();
        
        // Command turret (no additional offsets — ShootingCalculator handles all of them)
        turret.setTargetAngle(turretTargetAngle);
    }
    
    /**
     * Sets shooter RPM from ShootingCalculator's interpolated calibration values.
     */
    private void setShooterWithTraining() {
        // ShootingCalculator already calculated RPM values from calibration tables
        double topRPM = shootingCalc.getTargetTopRPM();
        double bottomRPM = shootingCalc.getTargetBottomRPM();
        
        // Set shooter RPM
        shooter.setTargetRPM(topRPM, bottomRPM);
    }
    
    // STATUS DETERMINATION
    
    /**
     * Determines the current status message to display.
     */
    private String determineStatus() {
        // Handle green light pre-shift (3 sec before our shift)
        if (gameState.isGreenLightPreShift()) {
            double secsUntil = gameState.getSecondsUntilOurNextShift();
            return String.format("GREEN LIGHT! Shift in %.1fs", secsUntil);
        }
        
        // Handle head-back warning (5 sec before our shift)
        if (gameState.isHeadBackWarning()) {
            double secsUntil = gameState.getSecondsUntilOurNextShift();
            return String.format("HEAD BACK - Shift in %.1fs", secsUntil);
        }
        
        // Handle alliance inactive
        if (!allianceActive && !gameState.isForceShootEnabled()) {
            double timeUntil = gameState.getTimeUntilActive();
            return String.format("WAITING - Active in %.1fs", timeUntil);
        }
        
        // Show target type and moving status
        String targetStr = (currentTargetMode == TargetMode.TRENCH) ? "[TRENCH] " : "[HUB] ";
        String movingStr = isMoving ? "[MOVING] " : "";
        
        // Show progress toward ready
        if (!turretOnTarget && !shooterReady) {
            return targetStr + movingStr + "Aiming + Spinning Up";
        } else if (!turretOnTarget) {
            return targetStr + movingStr + "Aiming...";
        } else if (!shooterReady) {
            return targetStr + movingStr + "Spinning Up...";
        } else {
            return targetStr + movingStr + "READY!";
        }
    }
    
    // TELEMETRY
    
    /**
     * Publishes essential auto-shoot data to SmartDashboard.
     */
    private void publishTelemetry() {
        SmartDashboard.putBoolean("AutoShoot/ReadyToFire", isReadyToFire());
        SmartDashboard.putNumber("AutoShoot/Distance", distanceToTarget);
    }
    
    // STATUS METHODS
    
    /**
     * Check if the system is ready to fire.
     * @return true if turret aimed, shooter spun up, AND alliance active (or green light pre-shift)
     */
    public boolean isReadyToFire() {
        return shooterReady && 
               (allianceActive || gameState.isForceShootEnabled() || gameState.isGreenLightPreShift());
    }
    
    /**
     * Get the current distance to target.
     * @return Distance in meters
     */
    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    // COMMAND LIFECYCLE (continued)

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        stopBallHandling();
        if (leds != null) {
            leds.clearAction();
        }
        SmartDashboard.putString("AutoShoot/Status", interrupted ? "Interrupted" : "Ended");
    }

    @Override
    public boolean isFinished() {
        // Runs continuously until interrupted
        // Use command composition if you want it to end when ready
        return false;
    }
}
