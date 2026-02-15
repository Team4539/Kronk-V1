package frc.robot.util;

import java.io.*;
import java.util.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Training system for hub and trench shooting with shooting-on-the-fly support.
 * Learns turret angle corrections and shooter power adjustments from manual corrections.
 * Includes battery voltage compensation and robot velocity tracking for lead calculation.
 * 
 * SHOOTING ON THE FLY:
 * When the robot is moving, the ball inherits the robot's velocity. To compensate:
 * - Track robot velocity (vX, vY, omega) when shots are made
 * - Calculate lead angle based on robot speed and direction relative to target
 * - Learn from successful shots while moving to improve predictions
 * 
 * HOW TO USE:
 * 1. Drive robot (can be moving!), aim at hub or trench
 * 2. Manually adjust turret angle until on target
 * 3. Manually adjust shooter powers until shots land correctly
 * 4. Call recordTrainingPoint() - this puts a CSV line on SmartDashboard
 * 5. Copy from "ShootingTraining/CopyThisLine" 
 * 6. Paste into src/main/deploy/shooting_training_data.csv
 * 7. Redeploy - robot now knows that correction
 * 
 * DATA TRACKED:
 * - Target type (HUB or TRENCH)
 * - Distance to target (meters)
 * - Robot velocity (vX, vY in m/s, omega in rad/s)
 * - Robot pose and turret angle
 * - Turret angle correction (degrees, + = right, - = left)
 * - Top motor power correction (+ = more power, - = less)
 * - Bottom motor power correction (+ = more power, - = less)
 * - Battery voltage at time of shot (for compensation)
 * - Shot result (HIT, SHORT, LONG, LEFT, RIGHT)
 * 
 * VOLTAGE COMPENSATION:
 * Power corrections are normalized to a reference voltage (12.5V).
 * When current voltage is lower, power corrections are scaled up proportionally.
 * 
 * VELOCITY COMPENSATION:
 * Lead angle is calculated based on robot velocity perpendicular to target.
 * Ball flight time is estimated from distance, then lateral velocity creates lead.
 */
public class ShootingTrainingManager {
    
    private static ShootingTrainingManager instance;
    
    /** Training data loaded from CSV */
    private final List<ShootingTrainingPoint> trainingPoints = new ArrayList<>();
    
    /** Last recorded line for copy-paste */
    private String lastRecordedLine = "";
    
    /** Reference voltage for normalization (fully charged battery under light load) */
    public static final double REFERENCE_VOLTAGE = 12.5;
    
    /** Minimum voltage to consider valid (below this, something is wrong) */
    public static final double MIN_VALID_VOLTAGE = 10.0;
    
    /** Maximum voltage compensation factor (don't over-compensate) */
    public static final double MAX_VOLTAGE_COMPENSATION = 1.25;
    
    // VELOCITY / SHOOTING ON THE FLY CONSTANTS
    
    /** Estimated average ball speed in m/s (used for lead angle calculation) */
    public static final double ESTIMATED_BALL_SPEED_MPS = 15.0;
    
    /** Minimum robot speed to consider "moving" in m/s */
    public static final double MIN_MOVING_SPEED_MPS = 0.3;
    
    /** Maximum lead angle to apply in degrees (safety limit) */
    public static final double MAX_LEAD_ANGLE_DEG = 25.0;
    
    /** Target types for shooting */
    public enum TargetType {
        HUB,
        TRENCH
    }
    
    /** Shot result for feedback */
    public enum ShotResult {
        HIT,      // Ball went in
        SHORT,    // Landed before target
        LONG,     // Went past target  
        LEFT,     // Missed left
        RIGHT     // Missed right
    }
    
    /**
     * A single shooting training data point.
     * Includes robot position, heading, velocity, and turret angle for context-aware predictions.
     * Velocity data enables shooting-on-the-fly lead compensation.
     */
    public static class ShootingTrainingPoint {
        public final TargetType targetType;
        public final double distanceMeters;
        public final double robotX;                 // Robot X position on field (meters)
        public final double robotY;                 // Robot Y position on field (meters)
        public final double robotHeadingDeg;        // Robot heading (degrees, 0 = toward red alliance)
        public final double robotVX;                // Robot X velocity (m/s, field-relative)
        public final double robotVY;                // Robot Y velocity (m/s, field-relative)
        public final double robotOmega;             // Robot rotational velocity (rad/s)
        public final double turretAngleDeg;         // Turret angle when shot was made (degrees)
        public final double angleToTargetDeg;       // Angle from robot to target (field-relative, degrees)
        public final double turretAngleCorrection;  // Degrees adjustment needed
        public final double topPowerCorrection;     // Power delta (-1 to 1)
        public final double bottomPowerCorrection;  // Power delta (-1 to 1)
        public final double batteryVoltage;         // Voltage when shot was made
        public final ShotResult result;
        
        public ShootingTrainingPoint(TargetType targetType, double distanceMeters,
                                     double robotX, double robotY, double robotHeadingDeg,
                                     double robotVX, double robotVY, double robotOmega,
                                     double turretAngleDeg, double angleToTargetDeg,
                                     double turretAngleCorrection, 
                                     double topPowerCorrection, double bottomPowerCorrection,
                                     double batteryVoltage,
                                     ShotResult result) {
            this.targetType = targetType;
            this.distanceMeters = distanceMeters;
            this.robotX = robotX;
            this.robotY = robotY;
            this.robotHeadingDeg = robotHeadingDeg;
            this.robotVX = robotVX;
            this.robotVY = robotVY;
            this.robotOmega = robotOmega;
            this.turretAngleDeg = turretAngleDeg;
            this.angleToTargetDeg = angleToTargetDeg;
            this.turretAngleCorrection = turretAngleCorrection;
            this.topPowerCorrection = topPowerCorrection;
            this.bottomPowerCorrection = bottomPowerCorrection;
            this.batteryVoltage = batteryVoltage;
            this.result = result;
        }
        
        /** Format for CSV file (15 fields with velocity data) */
        public String toCSV() {
            return String.format("%s,%.2f,%.2f,%.2f,%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.2f,%.3f,%.3f,%.2f,%s",
                targetType.name(),
                distanceMeters,
                robotX,
                robotY,
                robotHeadingDeg,
                robotVX,
                robotVY,
                robotOmega,
                turretAngleDeg,
                angleToTargetDeg,
                turretAngleCorrection,
                topPowerCorrection,
                bottomPowerCorrection,
                batteryVoltage,
                result.name());
        }
        
        /**
         * Get the robot's speed in m/s (magnitude of velocity).
         */
        public double getRobotSpeed() {
            return Math.sqrt(robotVX * robotVX + robotVY * robotVY);
        }
        
        /**
         * Check if robot was moving when this shot was taken.
         */
        public boolean wasMoving() {
            return getRobotSpeed() >= MIN_MOVING_SPEED_MPS;
        }
        
        /**
         * Get top power correction normalized to reference voltage.
         * This is what was needed at reference voltage to make the shot.
         */
        public double getNormalizedTopPowerCorrection() {
            // If shot was made at lower voltage, we needed more power
            // So the "true" correction at reference voltage is less
            double voltageRatio = batteryVoltage / REFERENCE_VOLTAGE;
            return topPowerCorrection * voltageRatio;
        }
        
        /**
         * Get bottom power correction normalized to reference voltage.
         */
        public double getNormalizedBottomPowerCorrection() {
            double voltageRatio = batteryVoltage / REFERENCE_VOLTAGE;
            return bottomPowerCorrection * voltageRatio;
        }
        
        /**
         * Calculate similarity score to another point based on position/angle/velocity.
         * Higher score = more similar conditions.
         * Includes velocity matching for shooting-on-the-fly accuracy.
         * 
         * @param distance Current distance to target
         * @param turretAngle Current turret angle
         * @param angleToTarget Current field-relative angle to target
         * @param vX Current robot X velocity (m/s)
         * @param vY Current robot Y velocity (m/s)
         * @return Similarity score (0 to 1)
         */
        public double getSimilarityScore(double distance, double turretAngle, double angleToTarget,
                                         double vX, double vY) {
            // Distance weight (most important - within 1.5m)
            double distanceDiff = Math.abs(this.distanceMeters - distance);
            double distanceScore = Math.max(0, 1.0 - distanceDiff / 1.5);
            
            // Turret angle weight (within 30 degrees)
            double turretDiff = Math.abs(this.turretAngleDeg - turretAngle);
            double turretScore = Math.max(0, 1.0 - turretDiff / 30.0);
            
            // Angle to target weight (within 20 degrees - captures left vs right side of hub)
            double angleDiff = Math.abs(this.angleToTargetDeg - angleToTarget);
            // Handle angle wrap-around
            if (angleDiff > 180) angleDiff = 360 - angleDiff;
            double angleScore = Math.max(0, 1.0 - angleDiff / 20.0);
            
            // Velocity similarity (within 1.5 m/s in each direction)
            double vxDiff = Math.abs(this.robotVX - vX);
            double vyDiff = Math.abs(this.robotVY - vY);
            double vxScore = Math.max(0, 1.0 - vxDiff / 1.5);
            double vyScore = Math.max(0, 1.0 - vyDiff / 1.5);
            double velocityScore = (vxScore + vyScore) / 2.0;
            
            // Check if both stationary or both moving
            double currentSpeed = Math.sqrt(vX * vX + vY * vY);
            boolean currentlyMoving = currentSpeed >= MIN_MOVING_SPEED_MPS;
            boolean wasMovingThen = wasMoving();
            
            // Penalize if motion state mismatch (one moving, one stationary)
            double motionMatchBonus = (currentlyMoving == wasMovingThen) ? 0.1 : -0.1;
            
            // Weighted combination: distance most important, then angle, then velocity, then turret
            double baseScore = distanceScore * 0.4 + angleScore * 0.25 + velocityScore * 0.2 + turretScore * 0.15;
            return Math.max(0, Math.min(1.0, baseScore + motionMatchBonus));
        }
        
        /**
         * Simple similarity score without velocity (for backwards compatibility).
         */
        public double getSimilarityScore(double distance, double turretAngle, double angleToTarget) {
            return getSimilarityScore(distance, turretAngle, angleToTarget, 0, 0);
        }
        
        /** Parse from CSV line - supports multiple formats */
        public static ShootingTrainingPoint fromCSV(String line) {
            try {
                String[] parts = line.split(",");
                
                // NEW format with velocity data (15 fields)
                // Format: Target,Distance,RobotX,RobotY,RobotHeading,VX,VY,Omega,TurretAngle,AngleToTarget,AngleCorr,TopPower,BottomPower,Voltage,Result
                if (parts.length >= 15) {
                    return new ShootingTrainingPoint(
                        TargetType.valueOf(parts[0].trim().toUpperCase()),
                        Double.parseDouble(parts[1].trim()),   // distance
                        Double.parseDouble(parts[2].trim()),   // robotX
                        Double.parseDouble(parts[3].trim()),   // robotY
                        Double.parseDouble(parts[4].trim()),   // robotHeading
                        Double.parseDouble(parts[5].trim()),   // vX
                        Double.parseDouble(parts[6].trim()),   // vY
                        Double.parseDouble(parts[7].trim()),   // omega
                        Double.parseDouble(parts[8].trim()),   // turretAngle
                        Double.parseDouble(parts[9].trim()),   // angleToTarget
                        Double.parseDouble(parts[10].trim()),  // angleCorrection
                        Double.parseDouble(parts[11].trim()),  // topPower
                        Double.parseDouble(parts[12].trim()),  // bottomPower
                        Double.parseDouble(parts[13].trim()),  // voltage
                        ShotResult.valueOf(parts[14].trim().toUpperCase())
                    );
                }
                // LEGACY format with pose but no velocity (12 fields)
                else if (parts.length >= 12) {
                    return new ShootingTrainingPoint(
                        TargetType.valueOf(parts[0].trim().toUpperCase()),
                        Double.parseDouble(parts[1].trim()),   // distance
                        Double.parseDouble(parts[2].trim()),   // robotX
                        Double.parseDouble(parts[3].trim()),   // robotY
                        Double.parseDouble(parts[4].trim()),   // robotHeading
                        0.0, 0.0, 0.0,                         // Unknown velocity - assume stationary
                        Double.parseDouble(parts[5].trim()),   // turretAngle
                        Double.parseDouble(parts[6].trim()),   // angleToTarget
                        Double.parseDouble(parts[7].trim()),   // angleCorrection
                        Double.parseDouble(parts[8].trim()),   // topPower
                        Double.parseDouble(parts[9].trim()),   // bottomPower
                        Double.parseDouble(parts[10].trim()),  // voltage
                        ShotResult.valueOf(parts[11].trim().toUpperCase())
                    );
                }
                // MEDIUM format with voltage but no pose (7 fields) - legacy
                else if (parts.length >= 7) {
                    return new ShootingTrainingPoint(
                        TargetType.valueOf(parts[0].trim().toUpperCase()),
                        Double.parseDouble(parts[1].trim()),   // distance
                        0.0, 0.0, 0.0,                         // Unknown pose
                        0.0, 0.0, 0.0,                         // Unknown velocity
                        0.0, 0.0,                              // Unknown turret/angle
                        Double.parseDouble(parts[2].trim()),   // angleCorrection
                        Double.parseDouble(parts[3].trim()),   // topPower
                        Double.parseDouble(parts[4].trim()),   // bottomPower
                        Double.parseDouble(parts[5].trim()),   // voltage
                        ShotResult.valueOf(parts[6].trim().toUpperCase())
                    );
                }
                // OLD format without voltage (6 fields) - legacy
                else if (parts.length >= 6) {
                    return new ShootingTrainingPoint(
                        TargetType.valueOf(parts[0].trim().toUpperCase()),
                        Double.parseDouble(parts[1].trim()),   // distance
                        0.0, 0.0, 0.0,                         // Unknown pose
                        0.0, 0.0, 0.0,                         // Unknown velocity
                        0.0, 0.0,                              // Unknown turret/angle
                        Double.parseDouble(parts[2].trim()),   // angleCorrection
                        Double.parseDouble(parts[3].trim()),   // topPower
                        Double.parseDouble(parts[4].trim()),   // bottomPower
                        REFERENCE_VOLTAGE,                      // Assume reference voltage
                        ShotResult.valueOf(parts[5].trim().toUpperCase())
                    );
                }
            } catch (Exception e) {
                // Skip bad lines
                System.out.println("ShootingTraining: Failed to parse line: " + line);
            }
            return null;
        }
    }
    
    public static ShootingTrainingManager getInstance() {
        if (instance == null) {
            instance = new ShootingTrainingManager();
        }
        return instance;
    }
    
    private ShootingTrainingManager() {
        loadFromCSV();
        updateDashboard();
    }
    
    /**
     * Get current battery voltage safely.
     */
    public static double getCurrentVoltage() {
        double voltage = RobotController.getBatteryVoltage();
        if (voltage < MIN_VALID_VOLTAGE) {
            return REFERENCE_VOLTAGE;  // Return reference if voltage seems invalid
        }
        return voltage;
    }
    
    /**
     * Calculate voltage compensation factor.
     * Returns how much to scale power corrections based on current voltage.
     * 
     * @return Compensation factor (>1 means boost power, <1 means reduce)
     */
    public static double getVoltageCompensation() {
        double currentVoltage = getCurrentVoltage();
        double compensation = REFERENCE_VOLTAGE / currentVoltage;
        // Clamp to avoid over-compensation
        return Math.min(compensation, MAX_VOLTAGE_COMPENSATION);
    }
    
    /**
     * Record a shooting training point with full pose and velocity data.
     * This is the primary recording method - captures all context for accurate predictions.
     * Velocity data enables shooting-on-the-fly predictions.
     * 
     * @param targetType HUB or TRENCH
     * @param distanceMeters Distance to target
     * @param robotX Robot X position on field (meters)
     * @param robotY Robot Y position on field (meters)
     * @param robotHeadingDeg Robot heading (degrees)
     * @param robotVX Robot X velocity (m/s, field-relative)
     * @param robotVY Robot Y velocity (m/s, field-relative)
     * @param robotOmega Robot rotational velocity (rad/s)
     * @param turretAngleDeg Turret angle (degrees)
     * @param angleToTargetDeg Field-relative angle from robot to target (degrees)
     * @param turretAngleCorrection How much turret angle was adjusted (+ = right)
     * @param topPowerCorrection How much top motor power was adjusted
     * @param bottomPowerCorrection How much bottom motor power was adjusted
     * @param batteryVoltage Battery voltage at time of shot
     * @param result What happened when you shot (HIT, SHORT, LONG, LEFT, RIGHT)
     */
    public void recordTrainingPoint(TargetType targetType, double distanceMeters,
                                    double robotX, double robotY, double robotHeadingDeg,
                                    double robotVX, double robotVY, double robotOmega,
                                    double turretAngleDeg, double angleToTargetDeg,
                                    double turretAngleCorrection,
                                    double topPowerCorrection, double bottomPowerCorrection,
                                    double batteryVoltage,
                                    ShotResult result) {
        ShootingTrainingPoint point = new ShootingTrainingPoint(
            targetType, distanceMeters, 
            robotX, robotY, robotHeadingDeg,
            robotVX, robotVY, robotOmega,
            turretAngleDeg, angleToTargetDeg,
            turretAngleCorrection, topPowerCorrection, bottomPowerCorrection,
            batteryVoltage, result);
        
        double robotSpeed = Math.sqrt(robotVX * robotVX + robotVY * robotVY);
        String movingStr = robotSpeed >= MIN_MOVING_SPEED_MPS ? " [MOVING %.1f m/s]" : " [STATIONARY]";
        
        lastRecordedLine = point.toCSV();
        
        // Put on SmartDashboard for easy copying
        SmartDashboard.putString("ShootingTraining/CopyThisLine", lastRecordedLine);
        SmartDashboard.putString("ShootingTraining/LastRecorded", 
            String.format("%s @ %.1fm (%.1f,%.1f) turret=%.0f° @ %.1fV v=(%.1f,%.1f): angle %+.1f°, top %+.3f, bottom %+.3f → %s",
                targetType.name(), distanceMeters, robotX, robotY, turretAngleDeg, batteryVoltage,
                robotVX, robotVY,
                turretAngleCorrection, topPowerCorrection, bottomPowerCorrection,
                result.name()));
        
        // Print to console
        System.out.println("");
        System.out.println("=== SHOOTING TRAINING POINT RECORDED ===");
        System.out.println(String.format("Target: %s @ %.2fm", targetType.name(), distanceMeters));
        System.out.println(String.format("Robot Pose: (%.2f, %.2f) heading=%.1f°", robotX, robotY, robotHeadingDeg));
        System.out.println(String.format("Robot Velocity: (%.2f, %.2f) m/s, omega=%.2f rad/s%s", 
            robotVX, robotVY, robotOmega, String.format(movingStr, robotSpeed)));
        System.out.println(String.format("Turret: %.1f° | Angle to target: %.1f°", turretAngleDeg, angleToTargetDeg));
        System.out.println(String.format("Battery: %.2fV", batteryVoltage));
        System.out.println("Copy this line into shooting_training_data.csv:");
        System.out.println(lastRecordedLine);
        System.out.println("==========================================");
        System.out.println("");
        
        // Add to current session
        trainingPoints.add(point);
        
        updateDashboard();
    }
    
    /**
     * Record a training point without velocity data (for stationary shots).
     * Convenience method that assumes zero velocity.
     */
    public void recordTrainingPoint(TargetType targetType, double distanceMeters,
                                    double robotX, double robotY, double robotHeadingDeg,
                                    double turretAngleDeg, double angleToTargetDeg,
                                    double turretAngleCorrection,
                                    double topPowerCorrection, double bottomPowerCorrection,
                                    double batteryVoltage,
                                    ShotResult result) {
        recordTrainingPoint(targetType, distanceMeters, robotX, robotY, robotHeadingDeg,
                           0.0, 0.0, 0.0,  // Zero velocity
                           turretAngleDeg, angleToTargetDeg, turretAngleCorrection,
                           topPowerCorrection, bottomPowerCorrection, batteryVoltage, result);
    }
    
    /**
     * Record a training point with simple SmartDashboard values.
     * Reads current values from SmartDashboard entries.
     * Automatically captures current battery voltage and velocity.
     */
    public void recordFromDashboard() {
        String targetStr = SmartDashboard.getString("ShootingTraining/Input/Target", "HUB");
        double distance = SmartDashboard.getNumber("ShootingTraining/Input/Distance", 3.0);
        double robotX = SmartDashboard.getNumber("ShootingTraining/Input/RobotX", 0.0);
        double robotY = SmartDashboard.getNumber("ShootingTraining/Input/RobotY", 0.0);
        double robotHeading = SmartDashboard.getNumber("ShootingTraining/Input/RobotHeading", 0.0);
        double robotVX = SmartDashboard.getNumber("ShootingTraining/Input/VelocityX", 0.0);
        double robotVY = SmartDashboard.getNumber("ShootingTraining/Input/VelocityY", 0.0);
        double robotOmega = SmartDashboard.getNumber("ShootingTraining/Input/Omega", 0.0);
        double turretAngle = SmartDashboard.getNumber("ShootingTraining/Input/TurretAngle", 0.0);
        double angleToTarget = SmartDashboard.getNumber("ShootingTraining/Input/AngleToTarget", 0.0);
        double angleCorr = SmartDashboard.getNumber("ShootingTraining/Input/AngleCorrection", 0.0);
        double topCorr = SmartDashboard.getNumber("ShootingTraining/Input/TopPowerCorrection", 0.0);
        double bottomCorr = SmartDashboard.getNumber("ShootingTraining/Input/BottomPowerCorrection", 0.0);
        String resultStr = SmartDashboard.getString("ShootingTraining/Input/Result", "HIT");
        
        TargetType target = TargetType.HUB;
        try {
            target = TargetType.valueOf(targetStr.toUpperCase());
        } catch (Exception e) {}
        
        ShotResult result = ShotResult.HIT;
        try {
            result = ShotResult.valueOf(resultStr.toUpperCase());
        } catch (Exception e) {}
        
        double voltage = getCurrentVoltage();
        recordTrainingPoint(target, distance, robotX, robotY, robotHeading, 
                           robotVX, robotVY, robotOmega,
                           turretAngle, angleToTarget, angleCorr, topCorr, bottomCorr, voltage, result);
    }
    
    // ===== LEAD ANGLE CALCULATION FOR SHOOTING ON THE FLY =====
    
    /**
     * Calculate the lead angle needed to compensate for robot movement.
     * When the robot is moving, the ball inherits that velocity and will drift.
     * We need to aim ahead of where the target currently is.
     * 
     * @param distanceMeters Distance to target
     * @param robotVX Robot X velocity (m/s, field-relative)
     * @param robotVY Robot Y velocity (m/s, field-relative)
     * @param angleToTargetDeg Direction to target (degrees, 0 = +X axis)
     * @return Lead angle to add to turret aim (degrees, + = aim right of target)
     */
    public static double calculateLeadAngle(double distanceMeters, double robotVX, double robotVY, double angleToTargetDeg) {
        // Estimate ball flight time based on distance and assumed ball speed
        double flightTime = distanceMeters / ESTIMATED_BALL_SPEED_MPS;
        
        // Convert angle to target to radians
        double angleRad = Math.toRadians(angleToTargetDeg);
        
        // Direction vector toward target
        double targetDirX = Math.cos(angleRad);
        double targetDirY = Math.sin(angleRad);
        
        // Perpendicular direction (to the right of target direction)
        double perpDirX = -targetDirY;  // Rotated 90 degrees clockwise
        double perpDirY = targetDirX;
        
        // Robot velocity component perpendicular to target direction
        // Positive = robot moving to the right relative to target
        double lateralVelocity = robotVX * perpDirX + robotVY * perpDirY;
        
        // Distance ball will drift laterally during flight
        double lateralDrift = lateralVelocity * flightTime;
        
        // Lead angle = arctan(drift / distance)
        double leadAngleRad = Math.atan2(lateralDrift, distanceMeters);
        double leadAngleDeg = Math.toDegrees(leadAngleRad);
        
        // Clamp to safety limits
        return Math.max(-MAX_LEAD_ANGLE_DEG, Math.min(MAX_LEAD_ANGLE_DEG, leadAngleDeg));
    }
    
    /**
     * Check if robot is currently moving fast enough to need lead compensation.
     * 
     * @param robotVX Robot X velocity (m/s)
     * @param robotVY Robot Y velocity (m/s)
     * @return true if robot speed exceeds minimum threshold
     */
    public static boolean isMovingForShot(double robotVX, double robotVY) {
        double speed = Math.sqrt(robotVX * robotVX + robotVY * robotVY);
        return speed >= MIN_MOVING_SPEED_MPS;
    }
    
    // ===== PREDICTION METHODS =====
    
    /**
     * Get predicted turret angle correction for shooting.
     * Uses similarity-weighted predictions based on distance, turret angle, and angle to target.
     * Only uses HIT data points for predictions.
     * 
     * @param targetType HUB or TRENCH
     * @param distanceMeters Current distance to target
     * @return Predicted angle correction in degrees
     */
    public double getPredictedAngleCorrection(TargetType targetType, double distanceMeters) {
        return getWeightedPrediction(targetType, distanceMeters, 0, 0, 0, 0, p -> p.turretAngleCorrection);
    }
    
    /**
     * Get predicted turret angle correction with full context including velocity.
     * Uses similarity-weighted predictions that account for shooting on the fly.
     * 
     * @param targetType HUB or TRENCH
     * @param distanceMeters Current distance to target
     * @param turretAngleDeg Current turret angle
     * @param angleToTargetDeg Current angle to target (field-relative)
     * @param robotVX Robot X velocity (m/s)
     * @param robotVY Robot Y velocity (m/s)
     * @return Predicted angle correction in degrees
     */
    public double getPredictedAngleCorrection(TargetType targetType, double distanceMeters,
                                               double turretAngleDeg, double angleToTargetDeg,
                                               double robotVX, double robotVY) {
        return getWeightedPrediction(targetType, distanceMeters, turretAngleDeg, angleToTargetDeg,
                                     robotVX, robotVY, p -> p.turretAngleCorrection);
    }
    
    /**
     * Get predicted top motor power correction WITH voltage compensation.
     * Uses normalized training data and scales for current battery voltage.
     * Only uses HIT data points for predictions.
     * 
     * @param targetType HUB or TRENCH
     * @param distanceMeters Current distance to target
     * @return Predicted power correction (already voltage-compensated)
     */
    public double getPredictedTopPowerCorrection(TargetType targetType, double distanceMeters) {
        return getPredictedTopPowerCorrection(targetType, distanceMeters, 0, 0, 0, 0);
    }
    
    /**
     * Get predicted top motor power correction with full context including velocity.
     */
    public double getPredictedTopPowerCorrection(TargetType targetType, double distanceMeters,
                                                  double turretAngleDeg, double angleToTargetDeg,
                                                  double robotVX, double robotVY) {
        double normalizedCorrection = getWeightedPrediction(targetType, distanceMeters, 
            turretAngleDeg, angleToTargetDeg, robotVX, robotVY, p -> p.getNormalizedTopPowerCorrection());
        return normalizedCorrection * getVoltageCompensation();
    }
    
    /**
     * Get predicted bottom motor power correction WITH voltage compensation.
     * Uses normalized training data and scales for current battery voltage.
     * Only uses HIT data points for predictions.
     * 
     * @param targetType HUB or TRENCH
     * @param distanceMeters Current distance to target
     * @return Predicted power correction (already voltage-compensated)
     */
    public double getPredictedBottomPowerCorrection(TargetType targetType, double distanceMeters) {
        return getPredictedBottomPowerCorrection(targetType, distanceMeters, 0, 0, 0, 0);
    }
    
    /**
     * Get predicted bottom motor power correction with full context including velocity.
     */
    public double getPredictedBottomPowerCorrection(TargetType targetType, double distanceMeters,
                                                     double turretAngleDeg, double angleToTargetDeg,
                                                     double robotVX, double robotVY) {
        double normalizedCorrection = getWeightedPrediction(targetType, distanceMeters, 
            turretAngleDeg, angleToTargetDeg, robotVX, robotVY, p -> p.getNormalizedBottomPowerCorrection());
        return normalizedCorrection * getVoltageCompensation();
    }
    
    /**
     * Get all predictions at once (more efficient than calling individually).
     * Power corrections are voltage-compensated.
     * 
     * @param targetType HUB or TRENCH
     * @param distanceMeters Current distance to target
     * @return double[3] = {angleCorrection, topPowerCorrection, bottomPowerCorrection}
     */
    public double[] getAllPredictions(TargetType targetType, double distanceMeters) {
        return new double[] {
            getPredictedAngleCorrection(targetType, distanceMeters),
            getPredictedTopPowerCorrection(targetType, distanceMeters),
            getPredictedBottomPowerCorrection(targetType, distanceMeters)
        };
    }
    
    /**
     * Get all predictions with full context including velocity for shooting on the fly.
     * 
     * @param targetType HUB or TRENCH
     * @param distanceMeters Current distance to target
     * @param turretAngleDeg Current turret angle
     * @param angleToTargetDeg Angle from robot to target
     * @param robotVX Robot X velocity (m/s)
     * @param robotVY Robot Y velocity (m/s)
     * @return double[3] = {angleCorrection, topPowerCorrection, bottomPowerCorrection}
     */
    public double[] getAllPredictions(TargetType targetType, double distanceMeters,
                                      double turretAngleDeg, double angleToTargetDeg,
                                      double robotVX, double robotVY) {
        return new double[] {
            getPredictedAngleCorrection(targetType, distanceMeters, turretAngleDeg, angleToTargetDeg, robotVX, robotVY),
            getPredictedTopPowerCorrection(targetType, distanceMeters, turretAngleDeg, angleToTargetDeg, robotVX, robotVY),
            getPredictedBottomPowerCorrection(targetType, distanceMeters, turretAngleDeg, angleToTargetDeg, robotVX, robotVY)
        };
    }
    
    /**
     * Weighted prediction using similarity scoring based on distance, turret angle, angle to target, and velocity.
     * Only uses HIT data points for prediction.
     */
    private double getWeightedPrediction(TargetType targetType, double distanceMeters,
                                         double turretAngleDeg, double angleToTargetDeg,
                                         double robotVX, double robotVY,
                                         java.util.function.ToDoubleFunction<ShootingTrainingPoint> valueExtractor) {
        List<ShootingTrainingPoint> relevant = new ArrayList<>();
        
        // Find HIT points for this target type
        for (ShootingTrainingPoint p : trainingPoints) {
            if (p.targetType == targetType && p.result == ShotResult.HIT) {
                // Use similarity score to filter - only include if reasonably similar
                double similarity = p.getSimilarityScore(distanceMeters, turretAngleDeg, angleToTargetDeg, robotVX, robotVY);
                if (similarity > 0.1) {  // At least 10% similar
                    relevant.add(p);
                }
            }
        }
        
        if (relevant.isEmpty()) {
            return 0.0;
        }
        
        // Weighted average based on similarity score
        double totalWeight = 0;
        double weightedSum = 0;
        
        for (ShootingTrainingPoint p : relevant) {
            double weight = p.getSimilarityScore(distanceMeters, turretAngleDeg, angleToTargetDeg, robotVX, robotVY);
            weight = Math.max(0.1, weight);
            
            weightedSum += valueExtractor.applyAsDouble(p) * weight;
            totalWeight += weight;
        }
        
        return weightedSum / totalWeight;
    }
    
    /**
     * Get confidence level (0 to 1) based on how many similar HIT data points we have.
     */
    public double getConfidence(TargetType targetType, double distanceMeters) {
        return getConfidence(targetType, distanceMeters, 0, 0, 0, 0);
    }
    
    /**
     * Get confidence level with full context including velocity.
     * Uses similarity scoring for more accurate confidence.
     */
    public double getConfidence(TargetType targetType, double distanceMeters,
                                double turretAngleDeg, double angleToTargetDeg,
                                double robotVX, double robotVY) {
        double totalSimilarity = 0;
        for (ShootingTrainingPoint p : trainingPoints) {
            if (p.targetType == targetType && p.result == ShotResult.HIT) {
                double similarity = p.getSimilarityScore(distanceMeters, turretAngleDeg, angleToTargetDeg, robotVX, robotVY);
                if (similarity > 0.1) {
                    totalSimilarity += similarity;
                }
            }
        }
        // 3+ high-similarity points = full confidence
        return Math.min(1.0, totalSimilarity / 3.0);
    }
    
    /**
     * Get number of HIT training points for a target type.
     */
    public int getHitCountForTarget(TargetType targetType) {
        int count = 0;
        for (ShootingTrainingPoint p : trainingPoints) {
            if (p.targetType == targetType && p.result == ShotResult.HIT) {
                count++;
            }
        }
        return count;
    }
    
    /**
     * Get total training points loaded.
     */
    public int getTotalPoints() {
        return trainingPoints.size();
    }
    
    // ===== TAG-BASED LOOKUP (for AimTurretToTagsCommand compatibility) =====
    
    /**
     * Converts an AprilTag ID to a target type based on field layout.
     * Uses Constants.Tags arrays to determine if tag is for hub or trench.
     * 
     * @param tagId The AprilTag ID
     * @return TargetType.HUB or TargetType.TRENCH (defaults to HUB if unknown)
     */
    public static TargetType getTargetTypeForTag(int tagId) {
        for (int id : Constants.Tags.BLUE_TRENCH) if (id == tagId) return TargetType.TRENCH;
        for (int id : Constants.Tags.RED_TRENCH) if (id == tagId) return TargetType.TRENCH;
        return TargetType.HUB; // Default to hub for hub tags or unknown tags
    }
    
    /**
     * Simple tag-based angle correction lookup.
     * Maps tag ID to target type and uses distance-based prediction.
     * For use by AimTurretToTagsCommand.
     * 
     * @param tagId The AprilTag being targeted
     * @param distance Distance to target in meters
     * @return Predicted angle correction in degrees
     */
    public double getPredictedCorrection(int tagId, double distance) {
        TargetType targetType = getTargetTypeForTag(tagId);
        return getPredictedAngleCorrection(targetType, distance);
    }
    
    /**
     * Simple tag-based confidence lookup.
     * 
     * @param tagId The AprilTag being targeted
     * @param distance Distance to target in meters
     * @return Confidence level 0-1
     */
    public double getConfidence(int tagId, double distance) {
        TargetType targetType = getTargetTypeForTag(tagId);
        return getConfidence(targetType, distance);
    }
    
    /**
     * Get count of training points for a tag (maps to target type).
     */
    public int getPointsForTag(int tagId) {
        TargetType targetType = getTargetTypeForTag(tagId);
        return getHitCountForTarget(targetType);
    }
    
    /**
     * Simple tag-based training point recording.
     * Records just an angle correction with minimal context.
     * Creates a minimal training point that can be used for predictions.
     * 
     * @param tagId The AprilTag being targeted
     * @param distance Distance to target in meters
     * @param angleCorrection How much turret angle was adjusted (+ = right)
     */
    public void recordTrainingPoint(int tagId, double distance, double angleCorrection) {
        TargetType targetType = getTargetTypeForTag(tagId);
        double voltage = getCurrentVoltage();
        
        // Create a minimal training point with just the essential data
        recordTrainingPoint(targetType, distance, 
            0, 0, 0,        // Unknown pose
            0, 0, 0,        // Stationary
            0, 0,           // Unknown turret/angle to target
            angleCorrection, 0, 0,  // Only angle correction, no power correction
            voltage, ShotResult.HIT);
    }
    
    /**
     * Reload data from CSV.
     */
    public void reload() {
        trainingPoints.clear();
        loadFromCSV();
        updateDashboard();
    }
    
    // Load training data from CSV
    private void loadFromCSV() {
        File csvFile = new File(Filesystem.getDeployDirectory(), "shooting_training_data.csv");
        
        if (!csvFile.exists()) {
            System.out.println("ShootingTraining: No CSV file found at " + csvFile.getPath());
            System.out.println("ShootingTraining: Create shooting_training_data.csv to enable learned corrections");
            return;
        }
        
        try (BufferedReader reader = new BufferedReader(new FileReader(csvFile))) {
            String line;
            int loaded = 0;
            int skipped = 0;
            
            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }
                
                ShootingTrainingPoint point = ShootingTrainingPoint.fromCSV(line);
                if (point != null) {
                    trainingPoints.add(point);
                    loaded++;
                } else {
                    skipped++;
                }
            }
            
            System.out.println("ShootingTraining: Loaded " + loaded + " points from CSV" + 
                              (skipped > 0 ? " (skipped " + skipped + " invalid lines)" : ""));
            
        } catch (IOException e) {
            System.out.println("ShootingTraining: Error reading CSV - " + e.getMessage());
        }
    }
    
    private void updateDashboard() {
        SmartDashboard.putNumber("ShootingTraining/TotalPoints", trainingPoints.size());
        SmartDashboard.putNumber("ShootingTraining/HubHits", getHitCountForTarget(TargetType.HUB));
        SmartDashboard.putNumber("ShootingTraining/TrenchHits", getHitCountForTarget(TargetType.TRENCH));
        SmartDashboard.putString("ShootingTraining/CopyThisLine", lastRecordedLine);
        
        // Voltage info
        SmartDashboard.putNumber("ShootingTraining/CurrentVoltage", getCurrentVoltage());
        SmartDashboard.putNumber("ShootingTraining/VoltageCompensation", getVoltageCompensation());
        SmartDashboard.putNumber("ShootingTraining/ReferenceVoltage", REFERENCE_VOLTAGE);
        
        // Shooting on the fly info
        SmartDashboard.putNumber("ShootingTraining/EstimatedBallSpeed", ESTIMATED_BALL_SPEED_MPS);
        SmartDashboard.putNumber("ShootingTraining/MinMovingSpeed", MIN_MOVING_SPEED_MPS);
        SmartDashboard.putNumber("ShootingTraining/MaxLeadAngle", MAX_LEAD_ANGLE_DEG);
        
        // Initialize input fields for easier recording (auto-filled by AutoShootCommand)
        SmartDashboard.putString("ShootingTraining/Input/Target", "HUB");
        SmartDashboard.putNumber("ShootingTraining/Input/Distance", 3.0);
        SmartDashboard.putNumber("ShootingTraining/Input/RobotX", 0.0);
        SmartDashboard.putNumber("ShootingTraining/Input/RobotY", 0.0);
        SmartDashboard.putNumber("ShootingTraining/Input/RobotHeading", 0.0);
        SmartDashboard.putNumber("ShootingTraining/Input/VelocityX", 0.0);
        SmartDashboard.putNumber("ShootingTraining/Input/VelocityY", 0.0);
        SmartDashboard.putNumber("ShootingTraining/Input/Omega", 0.0);
        SmartDashboard.putNumber("ShootingTraining/Input/TurretAngle", 0.0);
        SmartDashboard.putNumber("ShootingTraining/Input/AngleToTarget", 0.0);
        SmartDashboard.putNumber("ShootingTraining/Input/AngleCorrection", 0.0);
        SmartDashboard.putNumber("ShootingTraining/Input/TopPowerCorrection", 0.0);
        SmartDashboard.putNumber("ShootingTraining/Input/BottomPowerCorrection", 0.0);
        SmartDashboard.putString("ShootingTraining/Input/Result", "HIT");
        
        // Summary by target and distance ranges, plus moving vs stationary
        StringBuilder summary = new StringBuilder();
        
        int hubNear = 0, hubMid = 0, hubFar = 0, hubMoving = 0;
        int trenchNear = 0, trenchMid = 0, trenchFar = 0, trenchMoving = 0;
        
        for (ShootingTrainingPoint p : trainingPoints) {
            if (p.result != ShotResult.HIT) continue;
            
            if (p.targetType == TargetType.HUB) {
                if (p.distanceMeters < 3.0) hubNear++;
                else if (p.distanceMeters < 5.0) hubMid++;
                else hubFar++;
                if (p.wasMoving()) hubMoving++;
            } else {
                if (p.distanceMeters < 4.0) trenchNear++;
                else if (p.distanceMeters < 6.0) trenchMid++;
                else trenchFar++;
                if (p.wasMoving()) trenchMoving++;
            }
        }
        
        summary.append("HUB: <3m=").append(hubNear)
               .append(" 3-5m=").append(hubMid)
               .append(" >5m=").append(hubFar)
               .append(" moving=").append(hubMoving);
        summary.append(" | TRENCH: <4m=").append(trenchNear)
               .append(" 4-6m=").append(trenchMid)
               .append(" >6m=").append(trenchFar)
               .append(" moving=").append(trenchMoving);
        
        SmartDashboard.putString("ShootingTraining/Summary", summary.toString());
    }
}
