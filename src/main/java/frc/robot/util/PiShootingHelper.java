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
 * fallback: a fixed turret angle (0° = forward) and interpolated shooter powers
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
        
        boolean enabled = (targetMode != TargetMode.DISABLED);
        boolean robotenabled = DriverStation.isEnabled();
        
        // Convert chassis speeds to field-relative
        double headingRad = robotPose.getRotation().getRadians();
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double fieldVX = chassisSpeeds.vxMetersPerSecond * cosH - chassisSpeeds.vyMetersPerSecond * sinH;
        double fieldVY = chassisSpeeds.vxMetersPerSecond * sinH + chassisSpeeds.vyMetersPerSecond * cosH;
        
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
            // Pi is down - use fallback
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
    // TELEMETRY
    // ========================================================================
    
    private void publishTelemetry() {
        SmartDashboard.putBoolean("Pi/Connected", piConnected);
        SmartDashboard.putBoolean("Pi/UsingFallback", usingFallback);
    }
}
