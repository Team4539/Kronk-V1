package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.GameStateManager.TargetMode;

import java.util.Map;
import java.util.TreeMap;

/**
 * On-board shooting calculator - replaces the Raspberry Pi coprocessor.
 * 
 * Does everything locally on the roboRIO:
 *   1. Calculates turret angle to target using robot pose + turret offset
 *   2. Interpolates flywheel RPM from distance-based lookup tables
 *   3. Compensates for robot velocity (shooting on the fly)
 * 
 * This is simpler, faster (no NetworkTables lag), and more reliable than
 * offloading to a separate computer. Standard FRC approach used by top teams.
 * 
 * USAGE:
 *   Call update() every robot cycle with current pose and speeds.
 *   Read results via getTurretAngle(), getTargetTopRPM(), etc.
 */
public class ShootingCalculator {

    private static ShootingCalculator instance;

    // --- Cached output values ---
    
    /** Turret angle to command (degrees, -180 to +180, 0 = forward) */
    private double turretAngle = 0.0;
    
    /** Target top motor RPM */
    private double targetTopRPM = 0.0;
    
    /** Target bottom motor RPM */
    private double targetBottomRPM = 0.0;
    
    /** Distance from turret to target (meters) */
    private double distance = 0.0;
    
    /** Whether the robot is moving fast enough to need lead compensation */
    private boolean isMoving = false;
    
    /** Lead angle applied for moving shots (degrees) */
    private double leadAngle = 0.0;
    
    /** Whether we're aiming at trench (true) or hub (false) */
    private boolean aimingAtTrench = false;

    // --- Tuning constants ---
    
    /** Minimum robot speed (m/s) to apply lead compensation */
    private static final double MOVING_THRESHOLD_MPS = 0.3;
    
    /** 
     * Estimated time of flight for the game piece (seconds).
     * Used to calculate lead angle: how far the target "moves" relative
     * to us during the ball's flight time. Tune this based on testing.
     * Longer distances need more lead, so this scales with distance.
     */
    private static final double BASE_FLIGHT_TIME_SECONDS = 0.5;
    
    /** Flight time scaling factor per meter of distance */
    private static final double FLIGHT_TIME_PER_METER = 0.08;

    // --- References ---
    
    private final GameStateManager gameState = GameStateManager.getInstance();

    // ========================================================================
    // SINGLETON
    // ========================================================================

    public static ShootingCalculator getInstance() {
        if (instance == null) {
            instance = new ShootingCalculator();
        }
        return instance;
    }

    private ShootingCalculator() {
        System.out.println("[ShootingCalculator] Initialized - on-board shooting ready");
    }

    // ========================================================================
    // UPDATE (call every robot cycle)
    // ========================================================================

    /**
     * Recalculate the full shooting solution.
     * Call this every 20ms cycle from RobotContainer.updateVisionPose().
     * 
     * @param robotPose      Current robot pose on the field (from odometry + vision fusion)
     * @param chassisSpeeds   Current robot velocity (robot-relative)
     * @param targetMode      Current target mode (HUB, TRENCH, or DISABLED)
     * @param turretAngleOffset  Tunable turret angle offset from CalibrationManager (degrees)
     */
    public void update(Pose2d robotPose, ChassisSpeeds chassisSpeeds,
                       TargetMode targetMode, double turretAngleOffset) {
        
        if (targetMode == TargetMode.DISABLED) {
            turretAngle = 0.0;
            targetTopRPM = 0.0;
            targetBottomRPM = 0.0;
            distance = 0.0;
            isMoving = false;
            leadAngle = 0.0;
            return;
        }

        aimingAtTrench = (targetMode == TargetMode.TRENCH || gameState.isShuttleMode());
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        boolean isBlue = (alliance == Alliance.Blue);

        // --- 1. Get target position ---
        Translation2d targetPos = getTargetPosition(isBlue, aimingAtTrench);

        // --- 2. Calculate turret field position (accounts for turret offset on robot) ---
        double headingRad = robotPose.getRotation().getRadians();
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double turretX = robotPose.getX() 
            + Constants.Turret.TURRET_X_OFFSET * cosH 
            - Constants.Turret.TURRET_Y_OFFSET * sinH;
        double turretY = robotPose.getY() 
            + Constants.Turret.TURRET_X_OFFSET * sinH 
            + Constants.Turret.TURRET_Y_OFFSET * cosH;

        // --- 3. Calculate distance and base angle ---
        double dx = targetPos.getX() - turretX;
        double dy = targetPos.getY() - turretY;
        distance = Math.sqrt(dx * dx + dy * dy);
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        // --- 4. Convert field angle to robot-relative turret angle ---
        double robotHeading = robotPose.getRotation().getDegrees();
        turretAngle = fieldAngle - robotHeading;
        turretAngle = normalizeAngle(turretAngle);

        // --- 5. Lead compensation for shooting on the fly ---
        // Convert robot-relative speeds to field-relative
        double fieldVX = chassisSpeeds.vxMetersPerSecond * cosH - chassisSpeeds.vyMetersPerSecond * sinH;
        double fieldVY = chassisSpeeds.vxMetersPerSecond * sinH + chassisSpeeds.vyMetersPerSecond * cosH;
        double robotSpeed = Math.sqrt(fieldVX * fieldVX + fieldVY * fieldVY);
        
        isMoving = robotSpeed > MOVING_THRESHOLD_MPS;
        
        if (isMoving && distance > 0.5) {
            // Estimate flight time based on distance
            double flightTime = BASE_FLIGHT_TIME_SECONDS + (distance * FLIGHT_TIME_PER_METER);
            
            // Where will the robot be when the ball arrives? The ball inherits robot velocity,
            // but we need to lead the turret angle to compensate for the robot's lateral motion
            // relative to the target direction.
            
            // Project velocity onto perpendicular-to-target direction
            double targetAngleRad = Math.toRadians(fieldAngle);
            double perpVelocity = -fieldVX * Math.sin(targetAngleRad) + fieldVY * Math.cos(targetAngleRad);
            
            // Lead angle = atan(perpendicular_velocity * flight_time / distance)
            leadAngle = Math.toDegrees(Math.atan2(perpVelocity * flightTime, distance));
            
            // Apply lead angle
            turretAngle += leadAngle;
            turretAngle = normalizeAngle(turretAngle);
        } else {
            leadAngle = 0.0;
        }

        // --- 6. Apply turret angle offset ---
        turretAngle += turretAngleOffset;
        turretAngle = normalizeAngle(turretAngle);

        // --- 7. Interpolate shooter RPM from calibration table ---
        TreeMap<Double, double[]> calibration = aimingAtTrench
            ? Constants.Shooter.TRENCH_CALIBRATION
            : Constants.Shooter.SHOOTING_CALIBRATION;
        
        double[] rpms = interpolate(distance, calibration);
        double maxRPM = Constants.Shooter.MOTOR_FREE_SPEED_RPS * 60.0;
        targetTopRPM = clamp(rpms[0], 0, maxRPM);
        targetBottomRPM = clamp(rpms[1], 0, maxRPM);

        // --- 8. Publish telemetry ---
        publishTelemetry();
    }

    /**
     * Simplified update without turret angle offset.
     */
    public void update(Pose2d robotPose, ChassisSpeeds chassisSpeeds, TargetMode targetMode) {
        update(robotPose, chassisSpeeds, targetMode, 0.0);
    }

    // ========================================================================
    // TARGET POSITION
    // ========================================================================

    /**
     * Get the target position based on alliance and mode.
     */
    private Translation2d getTargetPosition(boolean isBlue, boolean trench) {
        if (trench) {
            return isBlue ? Constants.Field.BLUE_TRENCH_ROTATING : Constants.Field.RED_TRENCH_ROTATING;
        } else {
            return isBlue ? Constants.Field.BLUE_HUB_CENTER : Constants.Field.RED_HUB_CENTER;
        }
    }

    // ========================================================================
    // INTERPOLATION
    // ========================================================================

    /**
     * Linearly interpolates RPM values from a distance-keyed calibration table.
     * Clamps to boundary values if distance is outside the table range.
     */
    private double[] interpolate(double dist, TreeMap<Double, double[]> table) {
        Map.Entry<Double, double[]> lower = table.floorEntry(dist);
        Map.Entry<Double, double[]> upper = table.ceilingEntry(dist);

        if (lower == null && upper == null) return new double[]{3000, 3000};
        if (lower == null) return upper.getValue().clone();
        if (upper == null) return lower.getValue().clone();
        if (lower.getKey().equals(upper.getKey())) return lower.getValue().clone();

        double t = (dist - lower.getKey()) / (upper.getKey() - lower.getKey());
        return new double[]{
            lower.getValue()[0] + (upper.getValue()[0] - lower.getValue()[0]) * t,
            lower.getValue()[1] + (upper.getValue()[1] - lower.getValue()[1]) * t
        };
    }

    // ========================================================================
    // GETTERS
    // ========================================================================

    /** Turret angle to command (degrees, -180 to +180, 0 = forward). */
    public double getTurretAngle() { return turretAngle; }

    /** Target RPM for top shooter motor. */
    public double getTargetTopRPM() { return targetTopRPM; }

    /** Target RPM for bottom shooter motor. */
    public double getTargetBottomRPM() { return targetBottomRPM; }

    /** Distance from turret to target (meters). */
    public double getDistance() { return distance; }

    /** Whether robot is moving fast enough for lead compensation. */
    public boolean isMoving() { return isMoving; }

    /** Lead angle being applied for moving shots (degrees). */
    public double getLeadAngle() { return leadAngle; }

    /** Whether currently aiming at trench vs hub. */
    public boolean isAimingAtTrench() { return aimingAtTrench; }

    // ========================================================================
    // HELPERS
    // ========================================================================

    /** Normalize angle to [-180, +180). */
    private double normalizeAngle(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // ========================================================================
    // TELEMETRY
    // ========================================================================

    private void publishTelemetry() {
        SmartDashboard.putNumber("Shooting/Distance", distance);
        SmartDashboard.putNumber("Shooting/TopRPM", targetTopRPM);
        SmartDashboard.putNumber("Shooting/BottomRPM", targetBottomRPM);
    }
}
