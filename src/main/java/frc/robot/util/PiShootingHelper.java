package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.GameStateManager.TargetMode;

/**
 * Communicates with the Raspberry Pi shooting coprocessor via NetworkTables.
 * 
 * The Pi calculates the full shooting solution (turret angle, top speed, bottom speed)
 * based on robot position, velocity, and target mode. This class:
 *   1. Publishes robot state to Pi/Input
 *   2. Reads shooting solution from Pi/Output
 *   3. Falls back to known safe values if Pi is unreachable
 * 
 * FALLBACK BEHAVIOR:
 * If the Pi hasn't sent a heartbeat in the last 500ms, the robot uses a simple
 * fallback: a fixed turret angle (0 deg = forward) and interpolated shooter powers
 * from the calibration tables in Constants.java. This ensures the robot can still
 * shoot even if the Pi dies mid-match.
 */
public class PiShootingHelper {

    private static PiShootingHelper instance;

    // --- Connection monitoring ---
    
    /** How long without a heartbeat before we consider Pi disconnected (seconds) */
    private static final double CONNECTION_TIMEOUT_SECONDS = 0.5;
    
    /** Last heartbeat value received from Pi */
    private long lastHeartbeat = -1;
    
    /** Timer for tracking last heartbeat time */
    private double lastHeartbeatTime = 0.0;
    
    /** Whether we currently consider the Pi connected */
    private boolean piConnected = false;
    
    /** Count of consecutive cycles using fallback (for logging) */
    private int fallbackCycleCount = 0;
    
    // --- NetworkTables publishers (robot -> Pi) ---
    
    private final DoublePublisher pubRobotX;
    private final DoublePublisher pubRobotY;
    private final DoublePublisher pubRobotHeading;
    private final DoublePublisher pubRobotVX;
    private final DoublePublisher pubRobotVY;
    private final DoublePublisher pubRobotOmega;
    private final BooleanPublisher pubIsBlue;
    private final StringPublisher pubTargetMode;
    private final DoublePublisher pubBatteryVoltage;
    private final DoublePublisher pubHubOffsetX;
    private final DoublePublisher pubHubOffsetY;
    private final DoublePublisher pubTrenchOffsetX;
    private final DoublePublisher pubTrenchOffsetY;
    private final BooleanPublisher pubEnabled;
    private final BooleanPublisher pubRobotEnabled;
    
    // Training data recording (two-phase: snapshot at fire, confirm later)
    private final BooleanPublisher pubRecordShot;
    private final StringPublisher pubShotResult;
    
    // Snapshot publishers -- frozen state from the moment the ball was fired
    private final DoublePublisher pubSnapRobotX;
    private final DoublePublisher pubSnapRobotY;
    private final DoublePublisher pubSnapRobotHeading;
    private final DoublePublisher pubSnapRobotVX;
    private final DoublePublisher pubSnapRobotVY;
    private final DoublePublisher pubSnapRobotOmega;
    private final DoublePublisher pubSnapTurretAngle;
    private final DoublePublisher pubSnapTopSpeed;
    private final DoublePublisher pubSnapBottomSpeed;
    private final DoublePublisher pubSnapDistance;
    private final DoublePublisher pubSnapBatteryVoltage;
    private final StringPublisher pubSnapTargetMode;
    private final BooleanPublisher pubSnapValid;
    
    // --- NetworkTables subscribers (Pi -> robot) ---
    
    private final DoubleSubscriber subTurretAngle;
    private final DoubleSubscriber subTopSpeed;
    private final DoubleSubscriber subBottomSpeed;
    private final DoubleSubscriber subDistance;
    private final BooleanSubscriber subIsMoving;
    private final DoubleSubscriber subLeadAngle;
    private final DoubleSubscriber subConfidence;
    private final DoubleSubscriber subAngleCorrection;
    
    // Status subscribers
    @SuppressWarnings("unused")
    private final BooleanSubscriber subPiConnected;
    private final IntegerSubscriber subHeartbeat;
    private final BooleanSubscriber subModelLoaded;
    @SuppressWarnings("unused")
    private final DoubleSubscriber subLoopTimeMs;
    
    // --- Cached output values ---
    
    private double turretAngle = 0.0;
    private double topSpeed = 0.0;
    private double bottomSpeed = 0.0;
    private double distance = 0.0;
    private boolean isMoving = false;
    private double leadAngle = 0.0;
    private double confidence = 0.0;
    private double angleCorrection = 0.0;
    private boolean usingFallback = false;
    
    // --- Cached input values (for snapshot at fire time) ---
    
    private double lastRobotX = 0.0;
    private double lastRobotY = 0.0;
    private double lastRobotHeading = 0.0;
    private double lastFieldVX = 0.0;
    private double lastFieldVY = 0.0;
    private double lastOmega = 0.0;
    
    // --- References ---
    
    private final GameStateManager gameState = GameStateManager.getInstance();

    // ========================================================================
    // SINGLETON
    // ========================================================================
    
    public static PiShootingHelper getInstance() {
        if (instance == null) {
            instance = new PiShootingHelper();
        }
        return instance;
    }

    private PiShootingHelper() {
        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        
        // Input table (robot publishes for Pi to read)
        NetworkTable inputTable = ntInst.getTable("Pi/Input");
        pubRobotX = inputTable.getDoubleTopic("robot_x").publish();
        pubRobotY = inputTable.getDoubleTopic("robot_y").publish();
        pubRobotHeading = inputTable.getDoubleTopic("robot_heading").publish();
        pubRobotVX = inputTable.getDoubleTopic("robot_vx").publish();
        pubRobotVY = inputTable.getDoubleTopic("robot_vy").publish();
        pubRobotOmega = inputTable.getDoubleTopic("robot_omega").publish();
        pubIsBlue = inputTable.getBooleanTopic("is_blue_alliance").publish();
        pubTargetMode = inputTable.getStringTopic("target_mode").publish();
        pubBatteryVoltage = inputTable.getDoubleTopic("battery_voltage").publish();
        pubHubOffsetX = inputTable.getDoubleTopic("hub_offset_x").publish();
        pubHubOffsetY = inputTable.getDoubleTopic("hub_offset_y").publish();
        pubTrenchOffsetX = inputTable.getDoubleTopic("trench_offset_x").publish();
        pubTrenchOffsetY = inputTable.getDoubleTopic("trench_offset_y").publish();
        pubEnabled = inputTable.getBooleanTopic("enabled").publish();
        pubRobotEnabled = inputTable.getBooleanTopic("robot_enabled").publish();
        
        // Training recording
        pubRecordShot = inputTable.getBooleanTopic("record_shot").publish();
        pubShotResult = inputTable.getStringTopic("shot_result").publish();
        
        // Snapshot table -- frozen state from the moment the ball was fired
        // The Pi reads these when record_shot fires, instead of using its own last_shot_state
        NetworkTable snapTable = ntInst.getTable("Pi/Snapshot");
        pubSnapRobotX = snapTable.getDoubleTopic("robot_x").publish();
        pubSnapRobotY = snapTable.getDoubleTopic("robot_y").publish();
        pubSnapRobotHeading = snapTable.getDoubleTopic("robot_heading").publish();
        pubSnapRobotVX = snapTable.getDoubleTopic("robot_vx").publish();
        pubSnapRobotVY = snapTable.getDoubleTopic("robot_vy").publish();
        pubSnapRobotOmega = snapTable.getDoubleTopic("robot_omega").publish();
        pubSnapTurretAngle = snapTable.getDoubleTopic("turret_angle").publish();
        pubSnapTopSpeed = snapTable.getDoubleTopic("top_speed").publish();
        pubSnapBottomSpeed = snapTable.getDoubleTopic("bottom_speed").publish();
        pubSnapDistance = snapTable.getDoubleTopic("distance").publish();
        pubSnapBatteryVoltage = snapTable.getDoubleTopic("battery_voltage").publish();
        pubSnapTargetMode = snapTable.getStringTopic("target_mode").publish();
        pubSnapValid = snapTable.getBooleanTopic("valid").publish();
        pubSnapValid.set(false);
        
        // Output table (Pi publishes, robot reads)
        NetworkTable outputTable = ntInst.getTable("Pi/Output");
        subTurretAngle = outputTable.getDoubleTopic("turret_angle").subscribe(0.0);
        subTopSpeed = outputTable.getDoubleTopic("top_speed").subscribe(0.0);
        subBottomSpeed = outputTable.getDoubleTopic("bottom_speed").subscribe(0.0);
        subDistance = outputTable.getDoubleTopic("distance").subscribe(0.0);
        subIsMoving = outputTable.getBooleanTopic("is_moving").subscribe(false);
        subLeadAngle = outputTable.getDoubleTopic("lead_angle").subscribe(0.0);
        subConfidence = outputTable.getDoubleTopic("confidence").subscribe(0.0);
        subAngleCorrection = outputTable.getDoubleTopic("angle_correction").subscribe(0.0);
        
        // Status table
        NetworkTable statusTable = ntInst.getTable("Pi/Status");
        subPiConnected = statusTable.getBooleanTopic("connected").subscribe(false);
        subHeartbeat = statusTable.getIntegerTopic("heartbeat").subscribe(-1);
        subModelLoaded = statusTable.getBooleanTopic("model_loaded").subscribe(false);
        subLoopTimeMs = statusTable.getDoubleTopic("loop_time_ms").subscribe(0.0);
        
        System.out.println("[PiShootingHelper] Initialized - waiting for Pi connection");
    }

    // ========================================================================
    // UPDATE (call every robot cycle)
    // ========================================================================
    
    /**
     * Publish robot state to Pi and read the shooting solution.
     * Call this every robot cycle from AutoShootCommand or a subsystem periodic.
     * 
     * @param robotPose Current robot pose on field
     * @param chassisSpeeds Current robot velocity (robot-relative)
     * @param targetMode Current target mode (HUB, TRENCH, or DISABLED)
     * @param hubOffsetX Tunable hub X offset
     * @param hubOffsetY Tunable hub Y offset
     * @param trenchOffsetX Tunable trench X offset
     * @param trenchOffsetY Tunable trench Y offset
     */
    public void update(Pose2d robotPose, ChassisSpeeds chassisSpeeds,
                       TargetMode targetMode,
                       double hubOffsetX, double hubOffsetY,
                       double trenchOffsetX, double trenchOffsetY) {
        
        // Clear the record-shot pulse from the previous cycle
        if (needsClearRecordFlag) {
            pubRecordShot.set(false);
            needsClearRecordFlag = false;
        }
        
        boolean enabled = (targetMode != TargetMode.DISABLED);
        boolean robotenabled = DriverStation.isEnabled();
        
        // Convert chassis speeds to field-relative
        double headingRad = robotPose.getRotation().getRadians();
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double fieldVX = chassisSpeeds.vxMetersPerSecond * cosH - chassisSpeeds.vyMetersPerSecond * sinH;
        double fieldVY = chassisSpeeds.vxMetersPerSecond * sinH + chassisSpeeds.vyMetersPerSecond * cosH;
        
        // Cache input values for snapshot at fire time
        lastRobotX = robotPose.getX();
        lastRobotY = robotPose.getY();
        lastRobotHeading = robotPose.getRotation().getDegrees();
        lastFieldVX = fieldVX;
        lastFieldVY = fieldVY;
        lastOmega = chassisSpeeds.omegaRadiansPerSecond;
        
        // Publish inputs to Pi
        pubRobotX.set(robotPose.getX());
        pubRobotY.set(robotPose.getY());
        pubRobotHeading.set(robotPose.getRotation().getDegrees());
        pubRobotVX.set(fieldVX);
        pubRobotVY.set(fieldVY);
        pubRobotOmega.set(chassisSpeeds.omegaRadiansPerSecond);
        pubIsBlue.set(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);
        pubTargetMode.set(targetMode == TargetMode.TRENCH ? "TRENCH" : "HUB");
        pubBatteryVoltage.set(RobotController.getBatteryVoltage());
        pubHubOffsetX.set(hubOffsetX);
        pubHubOffsetY.set(hubOffsetY);
        pubTrenchOffsetX.set(trenchOffsetX);
        pubTrenchOffsetY.set(trenchOffsetY);
        pubEnabled.set(enabled);
        pubRobotEnabled.set(robotenabled);
        // Check Pi connection via heartbeat
        checkConnection();
        
        // Read outputs from Pi or use fallback
        if (piConnected && enabled) {
            // Pi is alive - use its solution
            turretAngle = subTurretAngle.get();
            topSpeed = subTopSpeed.get();
            bottomSpeed = subBottomSpeed.get();
            distance = subDistance.get();
            isMoving = subIsMoving.get();
            leadAngle = subLeadAngle.get();
            confidence = subConfidence.get();
            angleCorrection = subAngleCorrection.get();
            usingFallback = false;
            fallbackCycleCount = 0;
        } else if (enabled) {
            // Pi is down OR specifically requested - use fallback
            // IMPORTANT: Reset advanced tracking variables to prevent "ghost" moving shots
            isMoving = false;
            leadAngle = 0.0;
            angleCorrection = 0.0;
            confidence = 0.3; // Low confidence in fallback mode
            
            calculateFallback(robotPose, targetMode);
            usingFallback = true;
            fallbackCycleCount++;
            
            if (fallbackCycleCount == 1) {
                System.out.println("[PiShootingHelper] !! Pi disconnected - using FALLBACK shooting values");
            }
        } else {
            // Disabled - zero everything
            turretAngle = 0.0;
            topSpeed = 0.0;
            bottomSpeed = 0.0;
            distance = 0.0;
            isMoving = false;
            leadAngle = 0.0;
            confidence = 0.0;
            angleCorrection = 0.0;
            usingFallback = false;
        }
        
        // Publish telemetry
        publishTelemetry();
    }
    
    /**
     * Simplified update without tunable offsets.
     */
    public void update(Pose2d robotPose, ChassisSpeeds chassisSpeeds, TargetMode targetMode) {
        update(robotPose, chassisSpeeds, targetMode, 0, 0, 0, 0);
    }

    // ========================================================================
    // CONNECTION MONITORING
    // ========================================================================
    
    private void checkConnection() {
        long currentHeartbeat = subHeartbeat.get();
        double now = Timer.getFPGATimestamp();
        
        if (currentHeartbeat != lastHeartbeat) {
            // Got a new heartbeat
            lastHeartbeat = currentHeartbeat;
            lastHeartbeatTime = now;
            
            if (!piConnected) {
                piConnected = true;
                System.out.println("[PiShootingHelper] Pi CONNECTED (heartbeat=" + currentHeartbeat + ")");
            }
        }
        
        // Check for timeout
        if (piConnected && (now - lastHeartbeatTime) > CONNECTION_TIMEOUT_SECONDS) {
            piConnected = false;
            System.out.println("[PiShootingHelper] Pi DISCONNECTED (no heartbeat for " +
                String.format("%.1f", now - lastHeartbeatTime) + "s)");
        }
    }

    // ========================================================================
    // FALLBACK CALCULATION
    // ========================================================================
    
    /**
     * Simple fallback when Pi is unreachable.
     * Uses a fixed turret angle pointing at the target based on robot pose,
     * and interpolated shooter powers from the Constants calibration tables.
     */
    private void calculateFallback(Pose2d robotPose, TargetMode targetMode) {
        boolean isTrench = (targetMode == TargetMode.TRENCH || gameState.isShuttleMode());
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        boolean isBlue = (alliance == Alliance.Blue);
        
        // Get target position
        double targetX, targetY;
        if (isTrench) {
            if (isBlue) {
                targetX = Constants.Field.BLUE_TRENCH_ROTATING.getX();
                targetY = Constants.Field.BLUE_TRENCH_ROTATING.getY();
            } else {
                targetX = Constants.Field.RED_TRENCH_ROTATING.getX();
                targetY = Constants.Field.RED_TRENCH_ROTATING.getY();
            }
        } else {
            if (isBlue) {
                targetX = Constants.Field.BLUE_HUB_CENTER.getX();
                targetY = Constants.Field.BLUE_HUB_CENTER.getY();
            } else {
                targetX = Constants.Field.RED_HUB_CENTER.getX();
                targetY = Constants.Field.RED_HUB_CENTER.getY();
            }
        }
        
        // Calculate turret field position  
        double headingRad = robotPose.getRotation().getRadians();
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double turretX = robotPose.getX() + Constants.Turret.TURRET_X_OFFSET * cosH - Constants.Turret.TURRET_Y_OFFSET * sinH;
        double turretY = robotPose.getY() + Constants.Turret.TURRET_X_OFFSET * sinH + Constants.Turret.TURRET_Y_OFFSET * cosH;
        
        // Distance and angle
        double dx = targetX - turretX;
        double dy = targetY - turretY;
        distance = Math.sqrt(dx * dx + dy * dy);
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        
        // Convert to turret angle
        double robotHeading = robotPose.getRotation().getDegrees();
        turretAngle = fieldAngle - robotHeading;
        turretAngle = ((turretAngle + 180) % 360 + 360) % 360 - 180;
        
        // Interpolate shooter powers from calibration table
        java.util.TreeMap<Double, double[]> calibration = isTrench 
            ? Constants.Shooter.TRENCH_CALIBRATION 
            : Constants.Shooter.SHOOTING_CALIBRATION;
        
        java.util.Map.Entry<Double, double[]> lower = calibration.floorEntry(distance);
        java.util.Map.Entry<Double, double[]> upper = calibration.ceilingEntry(distance);
        
        if (lower == null && upper == null) {
            topSpeed = 0.5;
            bottomSpeed = 0.5;
        } else if (lower == null) {
            topSpeed = upper.getValue()[0];
            bottomSpeed = upper.getValue()[1];
        } else if (upper == null) {
            topSpeed = lower.getValue()[0];
            bottomSpeed = lower.getValue()[1];
        } else if (lower.getKey().equals(upper.getKey())) {
            topSpeed = lower.getValue()[0];
            bottomSpeed = lower.getValue()[1];
        } else {
            double t = (distance - lower.getKey()) / (upper.getKey() - lower.getKey());
            topSpeed = lower.getValue()[0] + (upper.getValue()[0] - lower.getValue()[0]) * t;
            bottomSpeed = lower.getValue()[1] + (upper.getValue()[1] - lower.getValue()[1]) * t;
        }
        
        // Clamp
        topSpeed = Math.max(0.0, Math.min(1.0, topSpeed));
        bottomSpeed = Math.max(0.0, Math.min(1.0, bottomSpeed));
        
        // No lead angle or training corrections in fallback
        isMoving = false;
        leadAngle = 0.0;
        angleCorrection = 0.0;
        confidence = 0.3;  // Low confidence in fallback mode
    }

    // ========================================================================
    // GETTERS (used by AutoShootCommand)
    // ========================================================================
    
    /** Get the turret angle to command (degrees, -180 to +180). */
    public double getTurretAngle() { return turretAngle; }
    
    /** Get top shooter motor power (0.0 to 1.0). */
    public double getTopSpeed() { return topSpeed; }
    
    /** Get bottom shooter motor power (0.0 to 1.0). */
    public double getBottomSpeed() { return bottomSpeed; }
    
    /** Get distance to target (meters). */
    public double getDistance() { return distance; }
    
    /** Whether robot is moving fast enough for lead compensation. */
    public boolean isMoving() { return isMoving; }
    
    /** Get the lead angle applied for moving shots (degrees). */
    public double getLeadAngle() { return leadAngle; }
    
    /** Get prediction confidence (0.0 to 1.0). */
    public double getConfidence() { return confidence; }
    
    /** Get the angle correction from trained model (degrees). */
    public double getAngleCorrection() { return angleCorrection; }
    
    /** Whether the Pi is currently connected and sending data. */
    public boolean isPiConnected() { return piConnected; }
    
    /** Whether we're currently using fallback values (Pi disconnected). */
    public boolean isUsingFallback() { return usingFallback; }
    
    /** Whether the Pi has a trained model loaded. */
    public boolean isPiModelLoaded() { return subModelLoaded.get(); }

    // ========================================================================
    // TRAINING DATA RECORDING (Two-Phase: Snapshot -> Confirm)
    // ========================================================================
    //
    // Phase 1: snapshotShotData() -- called automatically when ball is fired.
    //   Freezes all robot/shooter state at the exact moment of firing and
    //   publishes it to Pi/Snapshot. This data won't change until the next shot.
    //
    // Phase 2: confirmShot(result) -- called by operator AFTER watching the shot.
    //   Sends the result (HIT/MISS/LEFT/RIGHT/SHORT/LONG) to the Pi along with
    //   the frozen snapshot. The Pi writes the snapshot + result to CSV.
    //
    // This ensures training data always reflects the state at the moment the ball
    // left the shooter, not whenever the operator happened to press the button.
    // ========================================================================
    
    /** Whether there is a pending snapshot waiting for operator confirmation */
    private boolean hasPendingSnapshot = false;
    
    /** Timestamp when the snapshot was taken (for timeout/display) */
    private double snapshotTimestamp = 0.0;
    
    /** Flag to auto-clear the record shot pulse after one cycle */
    private boolean needsClearRecordFlag = false;
    
    /** How long a snapshot stays valid before expiring (seconds) */
    private static final double SNAPSHOT_TIMEOUT_SECONDS = 15.0;
    
    /**
     * Phase 1: Capture a snapshot of the current shooting state.
     * Called automatically by AutoShootCommand when the turret feed fires.
     * Freezes robot pose, shooter power, turret angle, etc. into Pi/Snapshot.
     * 
     * The snapshot stays valid until confirmed, discarded, or timed out.
     */
    public void snapshotShotData() {
        // Publish the frozen state to Pi/Snapshot table using cached values
        // from the most recent update() call (same 20ms cycle as the fire command)
        pubSnapRobotX.set(lastRobotX);
        pubSnapRobotY.set(lastRobotY);
        pubSnapRobotHeading.set(lastRobotHeading);
        pubSnapRobotVX.set(lastFieldVX);
        pubSnapRobotVY.set(lastFieldVY);
        pubSnapRobotOmega.set(lastOmega);
        
        // Use cached values (these are what the robot is actually commanding right now)
        pubSnapTurretAngle.set(turretAngle);
        pubSnapTopSpeed.set(topSpeed);
        pubSnapBottomSpeed.set(bottomSpeed);
        pubSnapDistance.set(distance);
        pubSnapBatteryVoltage.set(RobotController.getBatteryVoltage());
        pubSnapTargetMode.set(gameState.getTargetMode() == TargetMode.TRENCH ? "TRENCH" : "HUB");
        pubSnapValid.set(true);
        
        hasPendingSnapshot = true;
        snapshotTimestamp = Timer.getFPGATimestamp();
        
        // Also publish to dashboard so operator sees "Shot pending confirmation"
        SmartDashboard.putBoolean("Training/PendingShot", true);
        SmartDashboard.putNumber("Training/PendingDistance", distance);
        SmartDashboard.putString("Training/PendingMode", 
            gameState.getTargetMode() == TargetMode.TRENCH ? "TRENCH" : "HUB");
        
        System.out.println("[PiShootingHelper] Shot snapshot captured at d=" + 
            String.format("%.2f", distance) + "m -- waiting for operator confirmation");
    }
    
    /**
     * Overload that captures the snapshot using explicit pose/speed values.
     * Used when the caller has the exact pose at fire time.
     */
    public void snapshotShotData(Pose2d robotPose,
                                  ChassisSpeeds chassisSpeeds) {
        // Convert to field-relative velocities
        double headingRad = robotPose.getRotation().getRadians();
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double fieldVX = chassisSpeeds.vxMetersPerSecond * cosH - chassisSpeeds.vyMetersPerSecond * sinH;
        double fieldVY = chassisSpeeds.vxMetersPerSecond * sinH + chassisSpeeds.vyMetersPerSecond * cosH;
        
        pubSnapRobotX.set(robotPose.getX());
        pubSnapRobotY.set(robotPose.getY());
        pubSnapRobotHeading.set(robotPose.getRotation().getDegrees());
        pubSnapRobotVX.set(fieldVX);
        pubSnapRobotVY.set(fieldVY);
        pubSnapRobotOmega.set(chassisSpeeds.omegaRadiansPerSecond);
        
        pubSnapTurretAngle.set(turretAngle);
        pubSnapTopSpeed.set(topSpeed);
        pubSnapBottomSpeed.set(bottomSpeed);
        pubSnapDistance.set(distance);
        pubSnapBatteryVoltage.set(RobotController.getBatteryVoltage());
        pubSnapTargetMode.set(gameState.getTargetMode() == TargetMode.TRENCH ? "TRENCH" : "HUB");
        pubSnapValid.set(true);
        
        hasPendingSnapshot = true;
        snapshotTimestamp = Timer.getFPGATimestamp();
        
        SmartDashboard.putBoolean("Training/PendingShot", true);
        SmartDashboard.putNumber("Training/PendingDistance", distance);
        SmartDashboard.putString("Training/PendingMode", 
            gameState.getTargetMode() == TargetMode.TRENCH ? "TRENCH" : "HUB");
        
        System.out.println("[PiShootingHelper] Shot snapshot captured at d=" + 
            String.format("%.2f", distance) + "m -- waiting for operator confirmation");
    }
    
    /**
     * Phase 2: Confirm the pending shot with a result.
     * Called by operator after visually verifying the shot outcome.
     * Sends the result to the Pi, which pairs it with the frozen snapshot.
     * 
     * @param result Result string: "HIT", "MISS", "LEFT", "RIGHT", "SHORT", "LONG"
     * @return true if a pending snapshot was confirmed, false if nothing was pending
     */
    public boolean confirmShot(String result) {
        if (!hasPendingSnapshot) {
            System.out.println("[PiShootingHelper] No pending shot to confirm! Fire first, then confirm.");
            return false;
        }
        
        // Check for timeout
        double age = Timer.getFPGATimestamp() - snapshotTimestamp;
        if (age > SNAPSHOT_TIMEOUT_SECONDS) {
            System.out.println("[PiShootingHelper] Pending shot expired (" + 
                String.format("%.1f", age) + "s old). Discarding.");
            discardPendingShot();
            return false;
        }
        
        // Send result + trigger recording on Pi
        pubShotResult.set(result.toUpperCase());
        pubRecordShot.set(true);
        needsClearRecordFlag = true;
        
        hasPendingSnapshot = false;
        SmartDashboard.putBoolean("Training/PendingShot", false);
        
        System.out.println("[PiShootingHelper] Confirmed " + result.toUpperCase() + 
            " shot (" + String.format("%.1f", age) + "s after firing)");
        return true;
    }
    
    /**
     * Discard the pending snapshot without recording.
     * Use when operator decides not to log the shot (e.g., it was invalid).
     */
    public void discardPendingShot() {
        hasPendingSnapshot = false;
        pubSnapValid.set(false);
        SmartDashboard.putBoolean("Training/PendingShot", false);
        System.out.println("[PiShootingHelper] Pending shot discarded");
    }
    
    /** Whether there is a pending shot waiting for operator confirmation. */
    public boolean hasPendingShot() {
        // Auto-expire old snapshots
        if (hasPendingSnapshot && 
            (Timer.getFPGATimestamp() - snapshotTimestamp) > SNAPSHOT_TIMEOUT_SECONDS) {
            discardPendingShot();
        }
        return hasPendingSnapshot;
    }
    
    /** Get seconds since the pending snapshot was taken. */
    public double getPendingShotAge() {
        return hasPendingSnapshot ? Timer.getFPGATimestamp() - snapshotTimestamp : 0.0;
    }
    
    /**
     * Clear the recording flag. Called automatically by update().
     */
    public void clearRecordFlag() {
        pubRecordShot.set(false);
    }

    // ========================================================================
    // TELEMETRY
    // ========================================================================
    
    private void publishTelemetry() {
        SmartDashboard.putBoolean("Pi/Connected", piConnected);
        SmartDashboard.putBoolean("Pi/UsingFallback", usingFallback);
    }
}
