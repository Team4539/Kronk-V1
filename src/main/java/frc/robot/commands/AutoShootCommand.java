package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretFeedSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.ShootingTrainingManager;
import frc.robot.util.ShootingTrainingManager.TargetType;

/**
 * The big one! This command handles everything needed for fully automated shooting.
 * 
 * It combines turret aiming, shooter control, and ball feeding into one seamless
 * operation. The command is alliance-aware (only shoots when our window is active)
 * and can even do SHOOTING ON THE FLY with velocity-based lead compensation!
 * 
 * The ShootingTrainingManager provides learned corrections from practice sessions
 * to improve accuracy over time.
 */
public class AutoShootCommand extends Command {
    
    // SUBSYSTEMS & STATE MANAGER
    
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain; // For velocity data
    private final GameStateManager gameState;
    private final LEDSubsystem leds; // Optional - can be null
    private final SpindexerSubsystem spindexer; // Optional - can be null
    private final TurretFeedSubsystem turretFeed; // Optional - can be null
    private final ShootingTrainingManager trainingManager;
    
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
    
    /** Training corrections applied this frame */
    private double trainingAngleCorrection = 0.0;
    private double trainingTopPowerCorrection = 0.0;
    private double trainingBottomPowerCorrection = 0.0;
    private double trainingConfidence = 0.0;
    
    /** Current robot velocity for shooting on the fly */
    private double robotVX = 0.0;
    private double robotVY = 0.0;
    private double robotOmega = 0.0;
    
    /** Calculated lead angle for moving shots */
    private double leadAngle = 0.0;
    
    /** Whether currently moving fast enough to need lead compensation */
    private boolean isMoving = false;
    
    /** Tracks notification state to avoid spamming */
    private boolean hasNotifiedReady = false;
    private boolean wasAllianceActive = false;
    private boolean hasNotifiedHeadBack = false;
    private boolean hasNotifiedGreenLight = false;
    
    /** Real-time shot prediction (0-100%) */
    private double shotPrediction = 0.0;
    private String shotPredictionBreakdown = "";

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
        this(turret, shooter, limelight, null, null, null, null);
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
        this(turret, shooter, limelight, leds, null, null, null);
    }
    
    /**
     * Creates the full auto shoot command with LED feedback and ball handling.
     * When spindexer and turretFeed are provided, balls are automatically fed when ready.
     * NOTE: For shooting on the fly, pass drivetrain to get velocity data.
     * 
     * @param turret The turret subsystem for aiming
     * @param shooter The shooter subsystem for launching
     * @param limelight The limelight subsystem for distance/angle calculation
     * @param leds The LED subsystem for visual feedback (optional, can be null)
     * @param spindexer The spindexer subsystem for ball staging (optional, can be null)
     * @param turretFeed The turret feed subsystem for ball feeding (optional, can be null)
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter, LimelightSubsystem limelight, 
                           LEDSubsystem leds, SpindexerSubsystem spindexer, TurretFeedSubsystem turretFeed) {
        this(turret, shooter, limelight, leds, spindexer, turretFeed, null);
    }
    
    /**
     * Creates the full auto shoot command with shooting on the fly support.
     * When drivetrain is provided, lead angle compensation is applied based on robot velocity.
     * 
     * @param turret The turret subsystem for aiming
     * @param shooter The shooter subsystem for launching
     * @param limelight The limelight subsystem for distance/angle calculation
     * @param leds The LED subsystem for visual feedback (optional, can be null)
     * @param spindexer The spindexer subsystem for ball staging (optional, can be null)
     * @param turretFeed The turret feed subsystem for ball feeding (optional, can be null)
     * @param drivetrain The swerve drivetrain for velocity data (optional, can be null for no lead compensation)
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter, LimelightSubsystem limelight, 
                           LEDSubsystem leds, SpindexerSubsystem spindexer, TurretFeedSubsystem turretFeed,
                           CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.shooter = shooter;
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.leds = leds;
        this.spindexer = spindexer;
        this.turretFeed = turretFeed;
        this.gameState = GameStateManager.getInstance();
        this.trainingManager = ShootingTrainingManager.getInstance();
        
        // Requires turret and shooter, plus ball handling if provided
        addRequirements(turret, shooter);
        if (spindexer != null) {
            addRequirements(spindexer);
        }
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
        // Green light pre-shift (3 sec before our shift) overrides disabled - allow shooting!
        if (currentTargetMode == TargetMode.DISABLED && !gameState.isGreenLightPreShift()) {
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
        
        // ----- Calculate real-time shot prediction -----
        calculateShotPrediction();
        
        // ----- Check turret on-target status -----
        double turretError = Math.abs(turretTargetAngle - turret.getCurrentAngle());
        turretOnTarget = turretError < Constants.Turret.ON_TARGET_TOLERANCE_DEG;
        
        // ----- Check shooter ready status -----
        shooterReady = shooter.isReady();
        
        // ----- Control ball handling subsystems -----
        // Only feed balls when turret is aimed, shooter is ready, and alliance is active
        if (isReadyToFire()) {
            // Fire the balls!
            if (spindexer != null) {
                spindexer.setShoot();
            }
            if (turretFeed != null) {
                turretFeed.setShoot();
            }
        } else {
            // Not ready - keep ball handling in idle (staging mode)
            idleBallHandling();
        }
        
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
                .withTitle(" READY TO FIRE!")
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
                .withTitle(" Alliance Window OPEN")
                .withDescription("Scoring is now allowed!")
                .withDisplaySeconds(2.0));
            hasNotifiedHeadBack = false;
            hasNotifiedGreenLight = false;
        } else if (!allianceActive && wasAllianceActive && !gameState.isForceShootEnabled()) {
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle(" Alliance Window CLOSED")
                .withDescription("Waiting for next window...")
                .withDisplaySeconds(2.0));
        }
        wasAllianceActive = allianceActive;
        
        // Notify head-back warning (5 sec before our shift)
        if (gameState.isHeadBackWarning() && !hasNotifiedHeadBack) {
            hasNotifiedHeadBack = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle("⬅ HEAD BACK!")
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
                .withTitle(" GREEN LIGHT!")
                .withDescription("Cleared to shoot! Shift starts in " + 
                    String.format("%.0fs", gameState.getSecondsUntilOurNextShift()))
                .withDisplaySeconds(2.0));
        } else if (!gameState.isGreenLightPreShift()) {
            hasNotifiedGreenLight = false;
        }
    }
    
    /**
     * Updates LED action state based on current shooting status.
     */
    private void updateLEDState() {
        if (leds == null) return;
        
        // Determine priority action state
        if (turretOnTarget && shooterReady && (allianceActive || gameState.isGreenLightPreShift())) {
            // Ready to shoot! (including green light pre-shift window)
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
        if (spindexer != null) {
            spindexer.setIdle();
        }
        if (turretFeed != null) {
            turretFeed.setIdle();
        }
    }
    
    /**
     * Stops ball handling subsystems completely.
     */
    private void stopBallHandling() {
        if (spindexer != null) {
            spindexer.stop();
        }
        if (turretFeed != null) {
            turretFeed.stop();
        }
    }
    
    // AIMING LOGIC
    
    /**
     * Aims at the current target (hub or trench based on mode).
     * Applies learned training corrections for turret angle.
     * When moving, calculates and applies lead angle compensation.
     */
    private void aimAtTarget() {
        // Get robot velocity for shooting on the fly
        updateRobotVelocity();
        
        // Determine target type for training lookup
        TargetType targetType = (currentTargetMode == TargetMode.TRENCH || gameState.isShuttleMode()) 
            ? TargetType.TRENCH : TargetType.HUB;
        
        // Get distance and angle based on target mode
        double angleToTarget = 0.0;
        if (targetType == TargetType.TRENCH) {
            distanceToTarget = limelight.getDistanceToTrench();
            turretTargetAngle = limelight.getTurretAngleToTrench();
            angleToTarget = limelight.getAngleToTrench();
        } else {
            distanceToTarget = limelight.getDistanceToHub();
            turretTargetAngle = limelight.getTurretAngleToHub();
            angleToTarget = limelight.getAngleToHub();
        }
        
        // Apply global calibration offset (tunable via SmartDashboard)
        double calibrationOffset = SmartDashboard.getNumber("AutoShoot/TurretOffset", 
                                                            Constants.Turret.GLOBAL_ANGLE_OFFSET_DEG);
        turretTargetAngle += calibrationOffset;
        
        // Calculate lead angle for shooting on the fly
        isMoving = ShootingTrainingManager.isMovingForShot(robotVX, robotVY);
        if (isMoving) {
            leadAngle = ShootingTrainingManager.calculateLeadAngle(distanceToTarget, robotVX, robotVY, angleToTarget);
            turretTargetAngle += leadAngle;
        } else {
            leadAngle = 0.0;
        }
        
        // Apply learned training correction for turret angle (with velocity context)
        trainingAngleCorrection = trainingManager.getPredictedAngleCorrection(
            targetType, distanceToTarget, turret.getCurrentAngle(), angleToTarget, robotVX, robotVY);
        trainingConfidence = trainingManager.getConfidence(
            targetType, distanceToTarget, turret.getCurrentAngle(), angleToTarget, robotVX, robotVY);
        turretTargetAngle += trainingAngleCorrection;
        
        // Command turret
        turret.setTargetAngle(turretTargetAngle);
        
        // Update SmartDashboard for training data capture
        updateTrainingInputs(targetType, angleToTarget);
    }
    
    /**
     * Updates robot velocity from drivetrain (field-relative).
     * If drivetrain is null, assumes stationary.
     */
    private void updateRobotVelocity() {
        if (drivetrain != null) {
            ChassisSpeeds speeds = drivetrain.getState().Speeds;
            // Convert to field-relative using robot heading
            double headingRad = Math.toRadians(drivetrain.getState().Pose.getRotation().getDegrees());
            robotVX = speeds.vxMetersPerSecond * Math.cos(headingRad) - speeds.vyMetersPerSecond * Math.sin(headingRad);
            robotVY = speeds.vxMetersPerSecond * Math.sin(headingRad) + speeds.vyMetersPerSecond * Math.cos(headingRad);
            robotOmega = speeds.omegaRadiansPerSecond;
        } else {
            robotVX = 0.0;
            robotVY = 0.0;
            robotOmega = 0.0;
        }
    }
    
    /**
     * Updates SmartDashboard inputs for training data capture.
     * AutoShootCommand automatically fills in current state for easy recording.
     */
    private void updateTrainingInputs(TargetType targetType, double angleToTarget) {
        SmartDashboard.putString("ShootingTraining/Input/Target", targetType.name());
        SmartDashboard.putNumber("ShootingTraining/Input/Distance", distanceToTarget);
        SmartDashboard.putNumber("ShootingTraining/Input/VelocityX", robotVX);
        SmartDashboard.putNumber("ShootingTraining/Input/VelocityY", robotVY);
        SmartDashboard.putNumber("ShootingTraining/Input/Omega", robotOmega);
        SmartDashboard.putNumber("ShootingTraining/Input/TurretAngle", turret.getCurrentAngle());
        SmartDashboard.putNumber("ShootingTraining/Input/AngleToTarget", angleToTarget);
        
        if (drivetrain != null) {
            SmartDashboard.putNumber("ShootingTraining/Input/RobotX", drivetrain.getState().Pose.getX());
            SmartDashboard.putNumber("ShootingTraining/Input/RobotY", drivetrain.getState().Pose.getY());
            SmartDashboard.putNumber("ShootingTraining/Input/RobotHeading", drivetrain.getState().Pose.getRotation().getDegrees());
        }
    }
    
    /**
     * Sets shooter power with learned training corrections applied.
     */
    private void setShooterWithTraining() {
        // Determine target type for training lookup
        TargetType targetType = (currentTargetMode == TargetMode.TRENCH || gameState.isShuttleMode()) 
            ? TargetType.TRENCH : TargetType.HUB;
        
        // Get field-relative angle to target for velocity context
        double angleToTarget = (targetType == TargetType.TRENCH) 
            ? limelight.getAngleToTrench() 
            : limelight.getAngleToHub();
        
        // Get learned power corrections from training (with velocity context)
        trainingTopPowerCorrection = trainingManager.getPredictedTopPowerCorrection(
            targetType, distanceToTarget, turret.getCurrentAngle(), angleToTarget, robotVX, robotVY);
        trainingBottomPowerCorrection = trainingManager.getPredictedBottomPowerCorrection(
            targetType, distanceToTarget, turret.getCurrentAngle(), angleToTarget, robotVX, robotVY);
        
        // Set shooter power with training corrections applied (use correct calibration table)
        if (targetType == TargetType.TRENCH) {
            shooter.setForDistanceTrenchWithOffset(distanceToTarget, trainingTopPowerCorrection, trainingBottomPowerCorrection);
        } else {
            shooter.setForDistanceWithOffset(distanceToTarget, trainingTopPowerCorrection, trainingBottomPowerCorrection);
        }
    }
    
    // STATUS DETERMINATION
    
    /**
     * Determines the current status message to display.
     */
    private String determineStatus() {
        // Handle green light pre-shift (3 sec before our shift)
        if (gameState.isGreenLightPreShift()) {
            double secsUntil = gameState.getSecondsUntilOurNextShift();
            return String.format("GREEN LIGHT! Shift in %.1fs (%.0f%%)", secsUntil, shotPrediction);
        }
        
        // Handle head-back warning (5 sec before our shift)
        if (gameState.isHeadBackWarning()) {
            double secsUntil = gameState.getSecondsUntilOurNextShift();
            return String.format("HEAD BACK - Shift in %.1fs", secsUntil);
        }
        
        // Handle alliance inactive
        if (!allianceActive && !gameState.isForceShootEnabled()) {
            double timeUntil = gameState.getTimeUntilActive();
            return String.format("WAITING - Active in %.1fs (%.0f%% if now)", timeUntil, shotPrediction);
        }
        
        // Show target type, moving status, and prediction
        String targetStr = (currentTargetMode == TargetMode.TRENCH) ? "[TRENCH] " : "[HUB] ";
        String movingStr = isMoving ? "[MOVING] " : "";
        String predStr = String.format(" [%.0f%%]", shotPrediction);
        
        // Show progress toward ready
        if (!turretOnTarget && !shooterReady) {
            return targetStr + movingStr + "Aiming + Spinning Up" + predStr;
        } else if (!turretOnTarget) {
            return targetStr + movingStr + "Aiming..." + predStr;
        } else if (!shooterReady) {
            return targetStr + movingStr + "Spinning Up..." + predStr;
        } else {
            return targetStr + movingStr + "READY!" + predStr;
        }
    }
    
    // TELEMETRY
    
    /**
     * Calculates real-time shot success prediction (0-100%).
     * This runs continuously and shows "if you shot right now, here's your chance of making it."
     * 
     * Factors considered:
     * - Turret alignment (how close to target angle)
     * - Shooter readiness (speed at target)
     * - Training confidence (how much data we have for this situation)
     * - Robot stability (movement speed and rotation)
     * - Distance (optimal vs edge of range)
     */
    private void calculateShotPrediction() {
        StringBuilder breakdown = new StringBuilder();
        double totalScore = 0.0;
        
        // 1. TURRET ALIGNMENT (0-100%) - most important
        double turretError = Math.abs(turretTargetAngle - turret.getCurrentAngle());
        double turretScore;
        if (turretError < 0.5) {
            turretScore = 100.0; // Dead on
        } else if (turretError < Constants.Turret.ON_TARGET_TOLERANCE_DEG) {
            turretScore = 95.0 - (turretError * 2); // Small penalty
        } else if (turretError < 5.0) {
            turretScore = 80.0 - (turretError * 5); // Moderate penalty
        } else if (turretError < 15.0) {
            turretScore = 50.0 - (turretError * 2); // Large penalty
        } else {
            turretScore = 0.0; // Way off
        }
        turretScore = Math.max(0, Math.min(100, turretScore));
        breakdown.append(String.format("Aim:%.0f%% ", turretScore));
        totalScore += turretScore * 1.5; // Weight: 1.5x
        
        // 2. SHOOTER READINESS (0-100%)
        double shooterScore;
        if (shooter.isReady()) {
            shooterScore = 100.0;
        } else if (shooter.getTopPower() > 0.05) {
            shooterScore = 60.0; // Spinning up but not ready
        } else {
            shooterScore = 0.0; // Not running
        }
        breakdown.append(String.format("Shooter:%.0f%% ", shooterScore));
        totalScore += shooterScore * 1.2; // Weight: 1.2x
        
        // 3. TRAINING CONFIDENCE (0-100%) - how much we know about this shot
        double trainingScore = trainingConfidence * 100.0;
        // Even with no training, base accuracy from calibration tables
        trainingScore = Math.max(50, trainingScore); // At least 50% from calibration
        breakdown.append(String.format("Data:%.0f%% ", trainingScore));
        totalScore += trainingScore * 0.8; // Weight: 0.8x
        
        // 4. ROBOT STABILITY (0-100%) - penalize high speed/rotation
        double robotSpeed = Math.sqrt(robotVX * robotVX + robotVY * robotVY);
        double stabilityScore;
        if (robotSpeed < 0.1 && Math.abs(robotOmega) < 0.1) {
            stabilityScore = 100.0; // Stationary = best
        } else if (robotSpeed < 0.5) {
            stabilityScore = 95.0; // Barely moving
        } else if (robotSpeed < 1.0) {
            stabilityScore = 85.0; // Slow
        } else if (robotSpeed < 2.0) {
            stabilityScore = 70.0; // Medium - lead angle helps but adds uncertainty
        } else if (robotSpeed < 3.0) {
            stabilityScore = 55.0; // Fast
        } else {
            stabilityScore = 40.0; // Very fast - harder shot
        }
        // Penalize high rotation rate
        if (Math.abs(robotOmega) > 0.5) {
            stabilityScore -= 15;
        } else if (Math.abs(robotOmega) > 0.2) {
            stabilityScore -= 5;
        }
        stabilityScore = Math.max(0, stabilityScore);
        breakdown.append(String.format("Stable:%.0f%% ", stabilityScore));
        totalScore += stabilityScore * 1.0; // Weight: 1.0x
        
        // 5. DISTANCE QUALITY (0-100%) - optimal range vs too close/far
        double distanceScore;
        TargetType targetType = (currentTargetMode == TargetMode.TRENCH) ? TargetType.TRENCH : TargetType.HUB;
        if (targetType == TargetType.HUB) {
            // Hub optimal: 3-5m, acceptable: 2-7m
            if (distanceToTarget >= 3.0 && distanceToTarget <= 5.0) {
                distanceScore = 100.0;
            } else if (distanceToTarget >= 2.0 && distanceToTarget <= 7.0) {
                distanceScore = 80.0;
            } else if (distanceToTarget >= 1.5 && distanceToTarget <= 9.0) {
                distanceScore = 50.0;
            } else {
                distanceScore = 20.0; // Out of reliable range
            }
        } else {
            // Trench optimal: 4-6m, acceptable: 3-8m
            if (distanceToTarget >= 4.0 && distanceToTarget <= 6.0) {
                distanceScore = 100.0;
            } else if (distanceToTarget >= 3.0 && distanceToTarget <= 8.0) {
                distanceScore = 80.0;
            } else if (distanceToTarget >= 2.5 && distanceToTarget <= 10.0) {
                distanceScore = 50.0;
            } else {
                distanceScore = 20.0;
            }
        }
        breakdown.append(String.format("Dist:%.0f%%", distanceScore));
        totalScore += distanceScore * 0.5; // Weight: 0.5x
        
        // Calculate weighted average (weights sum to 5.0)
        double weightSum = 1.5 + 1.2 + 0.8 + 1.0 + 0.5;
        shotPrediction = totalScore / weightSum;
        shotPrediction = Math.max(0, Math.min(100, shotPrediction));
        
        shotPredictionBreakdown = breakdown.toString();
    }
    
    /**
     * Gets a human-readable prediction message.
     */
    private String getShotPredictionMessage() {
        if (shotPrediction >= 90) {
            return " EXCELLENT - Take the shot!";
        } else if (shotPrediction >= 75) {
            return "GOOD - High chance of success";
        } else if (shotPrediction >= 60) {
            return " FAIR - Might make it";
        } else if (shotPrediction >= 40) {
            return " RISKY - Consider repositioning";
        } else {
            return "POOR - Not recommended";
        }
    }

    /**
     * Publishes all auto-shoot data to SmartDashboard.
     */
    private void publishTelemetry() {
        // Main status indicators
        SmartDashboard.putBoolean("AutoShoot/ReadyToFire", isReadyToFire());
        SmartDashboard.putBoolean("AutoShoot/TurretOnTarget", turretOnTarget);
        SmartDashboard.putBoolean("AutoShoot/ShooterReady", shooterReady);
        SmartDashboard.putBoolean("AutoShoot/AllianceActive", allianceActive);
        
        // Target info
        SmartDashboard.putString("AutoShoot/TargetMode", currentTargetMode.toString());
        SmartDashboard.putNumber("AutoShoot/Distance", distanceToTarget);
        
        // Turret info
        SmartDashboard.putNumber("AutoShoot/TurretTarget", turretTargetAngle);
        SmartDashboard.putNumber("AutoShoot/TurretCurrent", turret.getCurrentAngle());
        SmartDashboard.putNumber("AutoShoot/TurretError", Math.abs(turretTargetAngle - turret.getCurrentAngle()));
        
        // Shooter info
        SmartDashboard.putNumber("AutoShoot/TopPower", shooter.getTopPower());
        SmartDashboard.putNumber("AutoShoot/BottomPower", shooter.getBottomPower());
        
        // Shooting on the fly info
        SmartDashboard.putBoolean("AutoShoot/IsMoving", isMoving);
        SmartDashboard.putNumber("AutoShoot/LeadAngle", leadAngle);
        SmartDashboard.putNumber("AutoShoot/RobotVX", robotVX);
        SmartDashboard.putNumber("AutoShoot/RobotVY", robotVY);
        SmartDashboard.putNumber("AutoShoot/RobotSpeed", Math.sqrt(robotVX * robotVX + robotVY * robotVY));
        
        // Training corrections applied (power corrections include voltage compensation)
        SmartDashboard.putNumber("AutoShoot/Training/AngleCorrection", trainingAngleCorrection);
        SmartDashboard.putNumber("AutoShoot/Training/TopPowerCorrection", trainingTopPowerCorrection);
        SmartDashboard.putNumber("AutoShoot/Training/BottomPowerCorrection", trainingBottomPowerCorrection);
        SmartDashboard.putNumber("AutoShoot/Training/Confidence", trainingConfidence);
        
        // Voltage compensation info
        SmartDashboard.putNumber("AutoShoot/Voltage/Current", ShootingTrainingManager.getCurrentVoltage());
        SmartDashboard.putNumber("AutoShoot/Voltage/Compensation", ShootingTrainingManager.getVoltageCompensation());
        
        // Timing info
        SmartDashboard.putNumber("AutoShoot/TimeRemainingActive", gameState.getTimeRemainingActive());
        
        // Editable turret offset
        double calibrationOffset = SmartDashboard.getNumber("AutoShoot/TurretOffset", 
                                                            Constants.Turret.GLOBAL_ANGLE_OFFSET_DEG);
        SmartDashboard.putNumber("AutoShoot/TurretOffset", calibrationOffset);
        
        // SHOT PREDICTION - "If you shoot now, here's your chance of making it"
        SmartDashboard.putNumber("AutoShoot/Prediction/Chance", shotPrediction);
        SmartDashboard.putString("AutoShoot/Prediction/Breakdown", shotPredictionBreakdown);
        SmartDashboard.putString("AutoShoot/Prediction/Message", getShotPredictionMessage());
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
    
    /**
     * Get the current shot prediction percentage.
     * This is continuously calculated based on aim, shooter, stability, distance, and training data.
     * @return Prediction 0-100% of making the shot if fired now
     */
    public double getShotPrediction() {
        return shotPrediction;
    }
    
    /**
     * Get the breakdown of shot prediction factors.
     * @return String like "Aim:95% Shooter:100% Data:80% Stable:100% Dist:100%"
     */
    public String getShotPredictionBreakdown() {
        return shotPredictionBreakdown;
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
