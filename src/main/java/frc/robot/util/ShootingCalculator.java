package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.GameStateManager.TargetMode;

import java.util.List;

/**
 * On-board shooting calculator for the fixed shooter.
 * 
 * Computes the shooting solution every cycle:
 *   1. Calculates distance and angle to target using robot pose
 *   2. Interpolates flywheel RPM from distance-based lookup tables
 * 
 * Since the shooter is fixed (no turret), aiming is done by rotating
 * the entire robot. The calculator still provides the angle-to-target
 * so the driver or auto can orient the robot correctly.
 * 
 * USAGE:
 *   Call update() every robot cycle with current pose and speeds.
 *   Read results via getAngleToTarget(), getTargetRPM(), etc.
 */
public class ShootingCalculator {

    private static ShootingCalculator instance;

    // --- Cached output values ---

    /** Angle from robot to target (degrees, -180 to +180, 0 = forward) */
    private double angleToTarget = 0;

    /** Target shooter motor RPM */
    private double targetRPM = 0.0;

    /** Distance from robot to target (meters) */
    private double distance = 0.0;

    /** Whether we're aiming at trench (true) or hub (false) */
    private boolean aimingAtTrench = false;

    /** Virtual (lead-compensated) target position for shoot-on-the-move */
    private Translation2d virtualTarget = new Translation2d();
    /** Estimated ball flight time in seconds */
    private double flightTimeSeconds = 0.0;
    /** Radial velocity component (m/s, positive = moving away from target) */
    private double radialVelocity = 0.0;

    // --- References ---
    
    private final GameStateManager gameState = GameStateManager.getInstance();
    
    /** 
     * When true, ALL baked-in offsets from Constants are bypassed.
     * Only live calibration slider values are applied.
     */
    private boolean calibrationMode = false;

    /** Field2d widget for displaying calibration points on SmartDashboard field */
    private final Field2d calibrationField = new Field2d();
    private boolean calibrationPointsNeedUpdate = true;
    private Translation2d lastPublishedTarget = null;

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
        SmartDashboard.putData("Shooting/CalibrationPoints", calibrationField);
        publishCalibrationPoints(Constants.Field.BLUE_HUB_CENTER);
        System.out.println("[ShootingCalculator] Initialized (fixed shooter)");
    }

    // ========================================================================
    // UPDATE (call every robot cycle)
    // ========================================================================

    /**
     * Recalculate the shooting solution for the fixed shooter.
     * Call this every 20ms cycle from RobotContainer.updateVisionPose().
     *
     * When shoot-on-the-move is enabled, the calculator compensates for the
     * robot's field-relative velocity by computing a "virtual target":
     *   1. Estimate ball flight time from distance and ball exit speed
     *   2. Virtual target = real target - (robot_velocity * flight_time)
     *   3. Aim angle and RPM are computed relative to the virtual target
     * This makes the ball land on the real target even while driving.
     *
     * @param robotPose     Current robot pose on the field
     * @param chassisSpeeds Current robot velocity (robot-relative)
     * @param targetMode    Current target mode (HUB, TRENCH, or DISABLED)
     * @param rpmOffset     RPM offset added to calculated RPM (positive = more power)
     */
    public void update(Pose2d robotPose, ChassisSpeeds chassisSpeeds,
                       TargetMode targetMode, double rpmOffset) {

        if (targetMode == TargetMode.DISABLED) {
            angleToTarget = 0.0;
            targetRPM = 0.0;
            distance = 0.0;
            flightTimeSeconds = 0.0;
            return;
        }

        // Guard against bad pose data (CAN glitch, uninitialized odometry)
        if (robotPose == null
                || Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY())
                || Double.isInfinite(robotPose.getX()) || Double.isInfinite(robotPose.getY())) {
            angleToTarget = 0.0;
            targetRPM = Constants.Shooter.FALLBACK_RPM;
            distance = 0.0;
            flightTimeSeconds = 0.0;
            return;
        }

        aimingAtTrench = (targetMode == TargetMode.TRENCH || gameState.isShuttleMode());
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        boolean isBlue = (alliance == Alliance.Blue);

        // --- 1. Get actual target position ---
        Translation2d targetPos = getTargetPosition(isBlue, aimingAtTrench);

        // --- 2. Compute static distance (used as seed for flight time) ---
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double dx = targetPos.getX() - robotX;
        double dy = targetPos.getY() - robotY;
        distance = Math.sqrt(dx * dx + dy * dy);

        // --- 3. Shoot-on-the-move: compute virtual (lead) target ---
        Translation2d aimTarget = targetPos;

        if (Constants.Shooter.SHOOT_ON_MOVE_ENABLED && chassisSpeeds != null) {
            // Convert robot-relative speeds to field-relative
            double headingRad = robotPose.getRotation().getRadians();
            double cos = Math.cos(headingRad);
            double sin = Math.sin(headingRad);
            double fieldVx = chassisSpeeds.vxMetersPerSecond * cos
                           - chassisSpeeds.vyMetersPerSecond * sin;
            double fieldVy = chassisSpeeds.vxMetersPerSecond * sin
                           + chassisSpeeds.vyMetersPerSecond * cos;

            double robotSpeed = Math.sqrt(fieldVx * fieldVx + fieldVy * fieldVy);

            // Only compensate if robot is actually moving (>0.1 m/s)
            if (robotSpeed > 0.1) {
                // Iteratively refine flight time and virtual target
                double estDist = distance;
                double exitAngleRad = Math.toRadians(Constants.Shooter.SHOOTER_EXIT_ANGLE_DEG);
                double cosExit = Math.cos(exitAngleRad);
                Translation2d leadTarget = targetPos;

                for (int i = 0; i < Constants.Shooter.FLIGHT_TIME_ITERATIONS; i++) {
                    // Estimate ball horizontal speed from RPM at this distance
                    double estRPM = calibrationMode ? rpmOffset
                            : interpolateRPM(estDist, Constants.Shooter.SHOOTING_CALIBRATION);
                    double wheelSurfaceSpeed = estRPM * 2.0 * Math.PI
                            * Constants.Shooter.WHEEL_RADIUS_METERS / 60.0;
                    double ballHorizontalSpeed = wheelSurfaceSpeed
                            * Constants.Shooter.BALL_SPEED_EFFICIENCY * cosExit;

                    // Flight time = distance / horizontal ball speed
                    if (ballHorizontalSpeed < 1.0) ballHorizontalSpeed = 1.0; // prevent div/0
                    flightTimeSeconds = estDist / ballHorizontalSpeed;

                    // Virtual target: offset by negative robot velocity * flight time
                    double compensationFactor = Constants.Shooter.MOVE_COMPENSATION_FACTOR;
                    leadTarget = new Translation2d(
                            targetPos.getX() - fieldVx * flightTimeSeconds * compensationFactor,
                            targetPos.getY() - fieldVy * flightTimeSeconds * compensationFactor);

                    // Recompute distance to the virtual target for next iteration
                    double ldx = leadTarget.getX() - robotX;
                    double ldy = leadTarget.getY() - robotY;
                    estDist = Math.sqrt(ldx * ldx + ldy * ldy);
                }

                aimTarget = leadTarget;
                // Update distance to the virtual target for RPM interpolation
                double ldx = aimTarget.getX() - robotX;
                double ldy = aimTarget.getY() - robotY;
                distance = Math.sqrt(ldx * ldx + ldy * ldy);

                // Compute radial velocity: positive = moving away from target
                // dot(velocity, unit vector from robot to target)
                double distToReal = Math.sqrt(dx * dx + dy * dy);
                if (distToReal > 0.01) {
                    radialVelocity = (fieldVx * dx + fieldVy * dy) / distToReal;
                    // Negate because dx/dy point FROM robot TO target,
                    // so positive dot = moving toward target = negative radial
                    radialVelocity = -radialVelocity;
                } else {
                    radialVelocity = 0.0;
                }
            } else {
                flightTimeSeconds = 0.0;
                radialVelocity = 0.0;
            }
        }

        virtualTarget = aimTarget;

        // --- 4. Calculate angle to (virtual) target ---
        double aimDx = aimTarget.getX() - robotX;
        double aimDy = aimTarget.getY() - robotY;
        double fieldAngle = Math.toDegrees(Math.atan2(aimDy, aimDx));

        // The shooter fires out the FRONT of the robot, so we aim the front
        // directly at the target. No 180° offset needed.
        double robotHeading = robotPose.getRotation().getDegrees();
        angleToTarget = normalizeAngle(fieldAngle - robotHeading);

        // --- 5. Interpolate shooter RPM from distance-based calibration table ---
        // In calibration mode, skip interpolation — use only the manual RPM offset
        // so the CalibrationManager slider directly controls the shooter.
        if (calibrationMode) {
            targetRPM = rpmOffset;
        } else {
            targetRPM = interpolateRPM(distance, Constants.Shooter.SHOOTING_CALIBRATION) + rpmOffset;

            // Radial RPM compensation: when driving away from target, the ball
            // must travel farther than the current distance by the time it arrives.
            // The virtual target handles aim direction, but the RPM table slope
            // may not fully account for the extra energy needed radially.
            // Positive radialVelocity = moving away = add RPM.
            targetRPM += radialVelocity * Constants.Shooter.RADIAL_RPM_PER_MPS;
        }
        double maxRPM = Constants.Shooter.MOTOR_FREE_SPEED_RPS * 60.0;
        targetRPM = clamp(targetRPM, 0, maxRPM);

        // --- 6. Publish telemetry ---
        publishTelemetry();
    }

    /**
     * Simplified update without RPM offset.
     */
    public void update(Pose2d robotPose, ChassisSpeeds chassisSpeeds, TargetMode targetMode) {
        update(robotPose, chassisSpeeds, targetMode, 0.0);
    }

    // ========================================================================
    // TARGET POSITION
    // ========================================================================

    private Translation2d getTargetPosition(boolean isBlue, boolean trench) {
        if (trench) {
            // Shuttle mode: aim at alliance zone side instead of trenches
            return isBlue ? Constants.Field.BLUE_SHUTTLE_TARGET : Constants.Field.RED_SHUTTLE_TARGET;
        } else {
            return isBlue ? Constants.Field.BLUE_HUB_CENTER : Constants.Field.RED_HUB_CENTER;
        }
    }

    // ========================================================================
    // INTERPOLATION
    // ========================================================================

    /**
     * Linearly interpolates (or extrapolates) RPM from a distance-based calibration table.
     * 
     * Each calibration point stores {distanceMeters, shooterRPM}.
     * Table should be sorted by distance (smallest first).
     * 
     * - Below the closest point: extrapolates using the slope of the first two points
     * - Above the farthest point: extrapolates using the slope of the last two points
     * - Between two points: linearly interpolates
     * 
     * Extrapolated RPM is clamped to a minimum of 0 to avoid negative values.
     * 
     * @param dist  Current distance to target (meters)
     * @param table List of {distanceMeters, shooterRPM}, sorted by distance
     * @return Interpolated (or extrapolated) shooter RPM
     */
    private double interpolateRPM(double dist, List<double[]> table) {
        if (table == null || table.isEmpty()) return 3000; // Fallback
        if (table.size() == 1) return table.get(0)[1];

        // Below the closest calibration point — extrapolate from first two points
        if (dist <= table.get(0)[0]) {
            double d0 = table.get(0)[0], rpm0 = table.get(0)[1];
            double d1 = table.get(1)[0], rpm1 = table.get(1)[1];
            double slope = (rpm1 - rpm0) / (d1 - d0);
            return Math.max(0, rpm0 + slope * (dist - d0));
        }

        // Above the farthest calibration point — extrapolate from last two points
        // Clamp to motor max RPM to prevent runaway extrapolation at long range
        int last = table.size() - 1;
        if (dist >= table.get(last)[0]) {
            double d0 = table.get(last - 1)[0], rpm0 = table.get(last - 1)[1];
            double d1 = table.get(last)[0],     rpm1 = table.get(last)[1];
            double slope = (rpm1 - rpm0) / (d1 - d0);
            double maxRPM = Constants.Shooter.MOTOR_FREE_SPEED_RPS * 60.0;
            return Math.max(0, Math.min(maxRPM, rpm1 + slope * (dist - d1)));
        }

        // Find the two bracketing points and lerp
        for (int i = 0; i < table.size() - 1; i++) {
            double d0 = table.get(i)[0];
            double d1 = table.get(i + 1)[0];
            if (dist >= d0 && dist <= d1) {
                double t = (dist - d0) / (d1 - d0);
                return table.get(i)[1] + t * (table.get(i + 1)[1] - table.get(i)[1]);
            }
        }
        // Shouldn't reach here, but fallback to last point
        return table.get(table.size() - 1)[1];
    }

    // ========================================================================
    // GETTERS
    // ========================================================================

    /** Angle from robot to target (degrees, -180 to +180, 0 = forward). 
     *  Use this to orient the robot toward the target for a fixed shooter. */
    public double getAngleToTarget() { return angleToTarget; }

    /** Target RPM for the shooter motor. */
    public double getTargetRPM() { return targetRPM; }

    /** Distance from robot to target (meters). */
    public double getDistance() { return distance; }

    /** Whether currently aiming at trench vs hub. */
    public boolean isAimingAtTrench() { return aimingAtTrench; }

    /** Estimated ball flight time (seconds). 0 when stationary. */
    public double getFlightTimeSeconds() { return flightTimeSeconds; }

    /** Virtual target position after shoot-on-the-move compensation. */
    public Translation2d getVirtualTarget() { return virtualTarget; }

    /** Whether calibration mode is active. */
    public boolean isCalibrationMode() { return calibrationMode; }
    
    /** Enable/disable calibration mode. */
    public void setCalibrationMode(boolean enabled) { 
        calibrationMode = enabled;
        System.out.println("[ShootingCalculator] Calibration mode " + (enabled ? "ENABLED" : "DISABLED"));
    }

    // ========================================================================
    // HELPERS
    // ========================================================================

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
        SmartDashboard.putNumber("Shooting/RPM", targetRPM);
        SmartDashboard.putNumber("Shooting/AngleToTarget", angleToTarget);
        SmartDashboard.putNumber("Shooting/FlightTime", flightTimeSeconds);
        SmartDashboard.putNumber("Shooting/RadialVelocity", radialVelocity);
        SmartDashboard.putNumber("Shooting/VirtualTargetX", virtualTarget.getX());
        SmartDashboard.putNumber("Shooting/VirtualTargetY", virtualTarget.getY());

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        boolean isBlue = (alliance == Alliance.Blue);
        Translation2d currentTarget = getTargetPosition(isBlue, aimingAtTrench);

        if (calibrationPointsNeedUpdate || lastPublishedTarget == null
                || !currentTarget.equals(lastPublishedTarget)) {
            publishCalibrationPoints(currentTarget);
        }
    }

    private void publishCalibrationPoints(Translation2d targetPos) {
        List<double[]> calTable = Constants.Shooter.SHOOTING_CALIBRATION;
        Pose2d[] fieldPoses = new Pose2d[calTable.size()];
        for (int i = 0; i < calTable.size(); i++) {
            double[] pt = calTable.get(i);
            double dist = pt[0];
            // Place calibration points in a ring around the target at this distance
            // (show them on the +X axis for visualization purposes)
            double fieldX = targetPos.getX() + dist;
            double fieldY = targetPos.getY();
            fieldPoses[i] = new Pose2d(fieldX, fieldY, Rotation2d.fromDegrees(0));
        }
        calibrationField.getObject("CalPoints").setPoses(fieldPoses);
        lastPublishedTarget = targetPos;
        calibrationPointsNeedUpdate = false;
    }
}
