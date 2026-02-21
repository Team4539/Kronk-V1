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

import java.util.List;

/**
 * On-board shooting calculator running on the roboRIO.
 * 
 * Computes the full shooting solution every cycle:
 *   1. Calculates turret angle to target using robot pose + turret offset
 *   2. Interpolates flywheel RPM from distance-based lookup tables
 *   3. Compensates for robot velocity (lead angle for shooting while moving)
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

    /** Robot-relative bearing to target BEFORE any offsets (degrees, -180 to +180).
     *  This captures the robot's orientation relative to the target direction.
     *  0 = robot facing target, 90 = target is to the robot's left, etc.
     *  Used as the second dimension in rotation offset calibration. */
    private double rawBearing = 0.0;

    /** Turret X position relative to target (meters). Positive = turret is in +X direction from target.
     *  Alliance-independent: computed as turretFieldPos - targetFieldPos each cycle. */
    private double relativeX = 0.0;

    /** Turret Y position relative to target (meters). Positive = turret is in +Y direction from target.
     *  Alliance-independent: computed as turretFieldPos - targetFieldPos each cycle. */
    private double relativeY = 0.0;

    // --- Tuning constants ---
    
    /** Minimum robot speed (m/s) to apply lead compensation */
    private static final double MOVING_THRESHOLD_MPS = 0.3;
    
    /** 
     * Estimated time of flight for the game piece (seconds).
     * Used to calculate lead angle: how far the target "moves" relative
     * to us during the ball's flight time. Tune this based on testing.
     * Longer distances need more lead, so this scales with distance.
     * 
     * For a ~0.215kg, 15cm ball launched from ~0.5m height at a target 1.44m high,
     * the ball follows a parabolic arc. This linear model is an approximation:
     *   flight_time ≈ BASE + distance × PER_METER
     * Validate with high-speed video or by measuring lead angle accuracy at speed.
     */
    private static final double BASE_FLIGHT_TIME_SECONDS = 0.5;
    
    /** Flight time scaling factor per meter of distance */
    private static final double FLIGHT_TIME_PER_METER = 0.08;

    // --- References ---
    
    private final GameStateManager gameState = GameStateManager.getInstance();
    
    /** 
     * When true, ALL baked-in offsets from Constants are bypassed.
     * Only live calibration slider values (turretAngleOffset, turretRotationOffset, 
     * topRPMOffset, bottomRPMOffset) are applied, so you're tuning from a raw baseline.
     * Set via CalibrationManager.startCalibration().
     */
    private boolean calibrationMode = false;

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
        System.out.println("[ShootingCalculator] Initialized");
    }

    // ========================================================================
    // UPDATE (call every robot cycle)
    // ========================================================================

    /**
     * Recalculate the full shooting solution.
     * Call this every 20ms cycle from RobotContainer.updateVisionPose().
     * 
     * @param robotPose         Current robot pose on the field (from odometry + vision fusion)
     * @param chassisSpeeds     Current robot velocity (robot-relative)
     * @param targetMode        Current target mode (HUB, TRENCH, or DISABLED)
     * @param turretAngleOffset Tunable turret angle offset from CalibrationManager (degrees)
     * @param topRPMOffset      RPM offset added to calculated top motor RPM (positive = more power)
     * @param bottomRPMOffset   RPM offset added to calculated bottom motor RPM (positive = more power)
     * @param turretRotationOffset Live turret rotation offset from CalibrationManager for calibration (degrees)
     */
    public void update(Pose2d robotPose, ChassisSpeeds chassisSpeeds,
                       TargetMode targetMode, double turretAngleOffset,
                       double topRPMOffset, double bottomRPMOffset,
                       double turretRotationOffset) {
        
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

        // Cache turret position relative to target (alliance-independent)
        // This is what gets stored in calibration points and used for interpolation.
        relativeX = turretX - targetPos.getX();
        relativeY = turretY - targetPos.getY();

        // --- 3. Calculate distance and base angle ---
        double dx = targetPos.getX() - turretX;
        double dy = targetPos.getY() - turretY;
        distance = Math.sqrt(dx * dx + dy * dy);
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        // --- 4. Convert field angle to robot-relative turret angle ---
        double robotHeading = robotPose.getRotation().getDegrees();
        turretAngle = fieldAngle - robotHeading;
        turretAngle = normalizeAngle(turretAngle);

        // Save the raw bearing (robot-relative angle to target) before any offsets or lead.
        // This captures the robot's orientation relative to the target for calibration.
        rawBearing = turretAngle;

        // --- 5. Lead compensation for shooting on the fly ---
        // The ball inherits the robot's velocity at launch. To hit a stationary
        // target, we need to aim so that the ball's inherited lateral drift is
        // cancelled by the turret angle correction.
        //
        // Physics: ball velocity = launch_velocity + robot_velocity
        // We want the ball to arrive at the target, so we adjust the turret
        // angle to subtract out the robot's perpendicular motion component.
        //
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
            // SUBTRACT because the ball inherits the robot's velocity. If the robot
            // is moving left (positive perpVelocity), the ball drifts left, so we
            // aim right (negative correction) to compensate.
            leadAngle = Math.toDegrees(Math.atan2(perpVelocity * flightTime, distance));
            
            // Apply lead angle (subtract — we aim opposite to the drift)
            turretAngle -= leadAngle;
            turretAngle = normalizeAngle(turretAngle);
        } else {
            leadAngle = 0.0;
        }

        // --- 6. Apply turret angle offsets ---
        if (calibrationMode) {
            // CALIBRATION MODE: Skip ALL baked-in Constants offsets.
            // Only the live slider values (turretAngleOffset, turretRotationOffset) are applied
            // so you're tuning from a raw, offset-free baseline.
            turretAngle += turretAngleOffset + turretRotationOffset;
        } else {
            // NORMAL MODE: Apply baked-in Constants offsets + live slider offsets.
            // Unified calibration provides the turret offset from the (relX, relY, bearing) table.
            double[] calResult = interpolateUnified(relativeX, relativeY, rawBearing,
                    Constants.Shooter.SHOOTING_CALIBRATION);
            double positionTurretOffset = calResult[0]; // turretOffsetDeg from table
            turretAngle += Constants.Turret.GLOBAL_ANGLE_OFFSET_DEG + turretAngleOffset
                    + positionTurretOffset + turretRotationOffset;
        }
        turretAngle = normalizeAngle(turretAngle);

        // --- 7. Interpolate shooter RPM from unified calibration table ---
        // The same table provides turret offset AND RPMs based on robot pose.
        double[] calResult = interpolateUnified(relativeX, relativeY, rawBearing,
                Constants.Shooter.SHOOTING_CALIBRATION);
        double maxRPM = Constants.Shooter.MOTOR_FREE_SPEED_RPS * 60.0;
        targetTopRPM = clamp(calResult[1] + topRPMOffset, 0, maxRPM);
        targetBottomRPM = clamp(calResult[2] + bottomRPMOffset, 0, maxRPM);

        // --- 8. Publish telemetry ---
        publishTelemetry();
    }

    /**
     * Update with turret angle offset but no RPM offsets or rotation offset.
     */
    public void update(Pose2d robotPose, ChassisSpeeds chassisSpeeds,
                       TargetMode targetMode, double turretAngleOffset) {
        update(robotPose, chassisSpeeds, targetMode, turretAngleOffset, 0.0, 0.0, 0.0);
    }

    /**
     * Simplified update without turret angle offset or RPM offsets.
     */
    public void update(Pose2d robotPose, ChassisSpeeds chassisSpeeds, TargetMode targetMode) {
        update(robotPose, chassisSpeeds, targetMode, 0.0, 0.0, 0.0, 0.0);
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
     * Interpolates turret offset + RPMs from the unified calibration table
     * using inverse-distance-weighted averaging in (relX, relY, bearing) space.
     * 
     * Each calibration point stores {relX, relY, bearingDeg, turretOffsetDeg, topRPM, bottomRPM}
     * where relX/relY are the turret position RELATIVE TO the target (turret - target).
     * This makes the table alliance-independent since the relative vector is the same
     * regardless of which alliance's target coordinates are used.
     * 
     * @param relX    Turret X relative to target (meters, turretX - targetX)
     * @param relY    Turret Y relative to target (meters, turretY - targetY)
     * @param bearing Robot-relative bearing to target (degrees, -180 to +180)
     * @param table   List of {relX, relY, bearingDeg, turretOffsetDeg, topRPM, bottomRPM}
     * @return double[3]: {turretOffsetDeg, topRPM, bottomRPM}
     */
    private double[] interpolateUnified(double relX, double relY, double bearing, List<double[]> table) {
        if (table == null || table.isEmpty()) return new double[]{0.0, 3000, 3000};
        if (table.size() == 1) {
            double[] pt = table.get(0);
            return new double[]{pt[3], pt[4], pt[5]};
        }

        double bearingWeight = Constants.Shooter.CALIBRATION_BEARING_WEIGHT;
        double totalWeight = 0.0;
        double weightedOffset = 0.0;
        double weightedTopRPM = 0.0;
        double weightedBottomRPM = 0.0;

        for (double[] point : table) {
            double dx = relX - point[0];
            double dy = relY - point[1];
            // Wrap bearing difference to [-180, 180] so -170° and 170° are 20° apart
            double dBearing = normalizeAngle(bearing - point[2]) * bearingWeight;
            double proximity = Math.sqrt(dx * dx + dy * dy + dBearing * dBearing);

            if (proximity < 0.001) {
                // Essentially on top of this calibration point — just return it
                return new double[]{point[3], point[4], point[5]};
            }

            double weight = 1.0 / (proximity * proximity); // Inverse-square weighting
            totalWeight += weight;
            weightedOffset += weight * point[3];
            weightedTopRPM += weight * point[4];
            weightedBottomRPM += weight * point[5];
        }

        return new double[]{
            weightedOffset / totalWeight,
            weightedTopRPM / totalWeight,
            weightedBottomRPM / totalWeight
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

    /** Robot-relative bearing to target before offsets (degrees, -180 to +180).
     *  0 = robot facing target, 90 = target to robot's left.
     *  Used for position-based rotation calibration. */
    public double getRawBearing() { return rawBearing; }

    /** Turret X position relative to target (meters). Alliance-independent. For calibration recording. */
    public double getRelativeX() { return relativeX; }

    /** Turret Y position relative to target (meters). Alliance-independent. For calibration recording. */
    public double getRelativeY() { return relativeY; }

    /** Whether calibration mode is active (baked-in Constants offsets bypassed). */
    public boolean isCalibrationMode() { return calibrationMode; }
    
    /** 
     * Enable/disable calibration mode.
     * When enabled, ALL baked-in offsets from Constants (GLOBAL_ANGLE_OFFSET_DEG, 
     * SHOOTING_CALIBRATION) are bypassed. Only live slider values apply.
     * Call this when starting/ending a calibration session.
     */
    public void setCalibrationMode(boolean enabled) { 
        calibrationMode = enabled;
        System.out.println("[ShootingCalculator] Calibration mode " + (enabled ? "ENABLED — baked-in offsets bypassed" : "DISABLED — using Constants offsets"));
    }

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
