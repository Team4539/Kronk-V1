package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.GameStateManager.TargetMode;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.ActionState;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretFeedSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.PiShootingHelper;

/**
 * The big one! This command handles everything needed for fully automated shooting.
 * 
 * It combines turret aiming, shooter control, and ball feeding into one seamless
 * operation. The command is alliance-aware (only shoots when our window is active)
 * and can even do SHOOTING ON THE FLY with velocity-based lead compensation!
 * 
 * All turret/shooter calculations are offloaded to the Raspberry Pi coprocessor
 * via PiShootingHelper. Falls back to calibration tables if Pi is unreachable.
 */
public class AutoShootCommand extends Command {
    
    // SUBSYSTEMS & STATE MANAGER
    
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final LimelightSubsystem limelight;
    @SuppressWarnings("unused")
    private final CommandSwerveDrivetrain drivetrain; // For velocity data (used by RobotContainer.updateVisionPose)
    private final GameStateManager gameState;
    private final LEDSubsystem leds; // Optional - can be null
    private final TurretFeedSubsystem turretFeed; // Optional - can be null
    private final PiShootingHelper piHelper;
    
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
     * Creates the auto shoot command (basic version without ball handling).
     * NOTE: For shooting on the fly, use the constructor with drivetrain.
     * 
     * @param turret The turret subsystem for aiming
     * @param shooter The shooter subsystem for launching
     * @param limelight The limelight subsystem for distance/angle calculation
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter, LimelightSubsystem limelight) {
        this(turret, shooter, limelight, null, null, null);
    }
    
    /**
     * Creates the auto shoot command with LED feedback but no ball handling.
     * 
     * @param turret The turret subsystem for aiming
     * @param shooter The shooter subsystem for launching
     * @param limelight The limelight subsystem for distance/angle calculation
     * @param leds The LED subsystem for visual feedback (optional, can be null)
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter, LimelightSubsystem limelight, LEDSubsystem leds) {
        this(turret, shooter, limelight, leds, null, null);
    }
    
    /**
     * Creates the full auto shoot command with LED feedback and ball handling.
     * When turretFeed is provided, balls are automatically fed when ready.
     * NOTE: For shooting on the fly, pass drivetrain to get velocity data.
     * 
     * @param turret The turret subsystem for aiming
     * @param shooter The shooter subsystem for launching
     * @param limelight The limelight subsystem for distance/angle calculation
     * @param leds The LED subsystem for visual feedback (optional, can be null)
     * @param turretFeed The turret feed subsystem for ball feeding (optional, can be null)
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter, LimelightSubsystem limelight, 
                           LEDSubsystem leds, TurretFeedSubsystem turretFeed) {
        this(turret, shooter, limelight, leds, turretFeed, null);
    }
    
    /**
     * Creates the full auto shoot command with shooting on the fly support.
     * When drivetrain is provided, lead angle compensation is applied based on robot velocity.
     * 
     * @param turret The turret subsystem for aiming
     * @param shooter The shooter subsystem for launching
     * @param limelight The limelight subsystem for distance/angle calculation
     * @param leds The LED subsystem for visual feedback (optional, can be null)
     * @param turretFeed The turret feed subsystem for ball feeding (optional, can be null)
     * @param drivetrain The swerve drivetrain for velocity data (optional, can be null for no lead compensation)
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter, LimelightSubsystem limelight, 
                           LEDSubsystem leds, TurretFeedSubsystem turretFeed,
                           CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.shooter = shooter;
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.leds = leds;
        this.turretFeed = turretFeed;
        this.gameState = GameStateManager.getInstance();
        this.piHelper = PiShootingHelper.getInstance();
        
        // Requires turret and shooter, plus ball handling if provided
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
        if (!limelight.hasPoseEstimate()) {
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
        
        // ----- Control ball handling subsystems -----
        // Only feed balls when turret is aimed, shooter is ready, and alliance is active
        boolean firing = isReadyToFire();
        if (firing) {
            // Fire the balls!
            if (turretFeed != null) {
                turretFeed.setShoot();
            }
            
            // Snapshot training data on the RISING EDGE (first cycle of firing).
            // This captures the exact robot state at the moment the ball is launched.
            if (!wasFiring) {
                piHelper.snapshotShotData();
            }
        } else {
            // Not ready - keep ball handling in idle (staging mode)
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
     * FIRING = balls actively being fed (highest priority visual)
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
    
    // BALL HANDLING CONTROL
    
    /**
     * Sets ball handling subsystems to idle mode (staging balls).
     */
    private void idleBallHandling() {
        if (turretFeed != null) {
            turretFeed.setIdle();
        }
    }
    
    /**
     * Stops ball handling subsystems completely.
     */
    private void stopBallHandling() {
        if (turretFeed != null) {
            turretFeed.stop();
        }
    }
    
    // AIMING LOGIC
    
    /**
     * Aims at the current target using the Raspberry Pi coprocessor.
     * The Pi calculates turret angle, lead compensation, and training corrections.
     * Falls back to local calculation if Pi is unreachable.
     * 
     * NOTE: piHelper.update() is called from RobotContainer.updateVisionPose() every cycle,
     * so we only READ the Pi's solution here -- we don't publish a second update.
     */
    private void aimAtTarget() {
        // Read Pi results (or fallback values) -- already updated by RobotContainer
        turretTargetAngle = piHelper.getTurretAngle();
        distanceToTarget = piHelper.getDistance();
        isMoving = piHelper.isMoving();
        
        // Apply global calibration offset (tunable via SmartDashboard)
        double calibrationOffset = SmartDashboard.getNumber("AutoShoot/TurretOffset", 
                                                            Constants.Turret.GLOBAL_ANGLE_OFFSET_DEG);
        turretTargetAngle += calibrationOffset;
        
        // Command turret
        turret.setTargetAngle(turretTargetAngle);
    }
    
    /**
     * Sets shooter power using the Pi coprocessor results.
     * The Pi provides final top and bottom motor power directly.
     * Falls back to calibration table interpolation if Pi is unreachable.
     */
    private void setShooterWithTraining() {
        // Pi already calculated final power values (including training corrections)
        double topPower = piHelper.getTopSpeed();
        double bottomPower = piHelper.getBottomSpeed();
        
        // Set shooter power directly
        shooter.setManualPower(topPower, bottomPower);
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
        String piStr = piHelper.isUsingFallback() ? "[FALLBACK] " : "";
        
        // Show progress toward ready
        if (!turretOnTarget && !shooterReady) {
            return targetStr + piStr + movingStr + "Aiming + Spinning Up";
        } else if (!turretOnTarget) {
            return targetStr + piStr + movingStr + "Aiming...";
        } else if (!shooterReady) {
            return targetStr + piStr + movingStr + "Spinning Up...";
        } else {
            return targetStr + piStr + movingStr + "READY!";
        }
    }
    
    // TELEMETRY
    
    /**
     * Publishes essential auto-shoot data to SmartDashboard.
     */
    private void publishTelemetry() {
        SmartDashboard.putBoolean("AutoShoot/ReadyToFire", isReadyToFire());
        SmartDashboard.putNumber("AutoShoot/Distance", distanceToTarget);
        
        // Editable turret offset
        double calibrationOffset = SmartDashboard.getNumber("AutoShoot/TurretOffset", 
                                                            Constants.Turret.GLOBAL_ANGLE_OFFSET_DEG);
        SmartDashboard.putNumber("AutoShoot/TurretOffset", calibrationOffset);
    }
    
    // STATUS METHODS
    
    /**
     * Check if the system is ready to fire.
     * @return true if turret aimed, shooter spun up, AND alliance active (or green light pre-shift)
     */
    public boolean isReadyToFire() {
        return turretOnTarget && shooterReady && 
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
