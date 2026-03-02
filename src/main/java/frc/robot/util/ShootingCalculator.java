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
    private double angleToTarget = 0.0;
    
    /** Target shooter motor RPM */
    private double targetRPM = 0.0;
    
    /** Distance from robot to target (meters) */
    private double distance = 0.0;
    
    /** Whether the robot is moving fast enough to need lead compensation */
    private boolean isMoving = false;
    
    /** Whether we're aiming at trench (true) or hub (false) */
    private boolean aimingAtTrench = false;

    /** Robot-relative bearing to target BEFORE any offsets (degrees, -180 to +180). */
    private double rawBearing = 0.0;

    /** Robot X position relative to target (meters). Alliance-independent. */
    private double relativeX = 0.0;

    /** Robot Y position relative to target (meters). Alliance-independent. */
    private double relativeY = 0.0;

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
            isMoving = false;
            return;
        }

        aimingAtTrench = (targetMode == TargetMode.TRENCH || gameState.isShuttleMode());
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        boolean isBlue = (alliance == Alliance.Blue);

        // --- 1. Get target position ---
        Translation2d targetPos = getTargetPosition(isBlue, aimingAtTrench);

        // --- 2. Calculate robot center position (no turret offset needed) ---
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        // Cache position relative to target (alliance-independent)
        relativeX = robotX - targetPos.getX();
        relativeY = robotY - targetPos.getY();

        // --- 3. Calculate distance and base angle ---
        double dx = targetPos.getX() - robotX;
        double dy = targetPos.getY() - robotY;
        distance = Math.sqrt(dx * dx + dy * dy);
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        // --- 4. Convert field angle to robot-relative angle ---
        double robotHeading = robotPose.getRotation().getDegrees();
        angleToTarget = fieldAngle - robotHeading;
        angleToTarget = normalizeAngle(angleToTarget);

        // Save the raw bearing before any offsets
        rawBearing = angleToTarget;

        // --- 5. Interpolate shooter RPM from calibration table ---
        double[] calResult = interpolateUnified(relativeX, relativeY, rawBearing,
                Constants.Shooter.SHOOTING_CALIBRATION);
        double maxRPM = Constants.Shooter.MOTOR_FREE_SPEED_RPS * 60.0;
        targetRPM = clamp(calResult[0] + rpmOffset, 0, maxRPM);

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
            return isBlue ? Constants.Field.BLUE_TRENCH_ROTATING : Constants.Field.RED_TRENCH_ROTATING;
        } else {
            return isBlue ? Constants.Field.BLUE_HUB_CENTER : Constants.Field.RED_HUB_CENTER;
        }
    }

    // ========================================================================
    // INTERPOLATION
    // ========================================================================

    /**
     * Interpolates RPM from the calibration table using inverse-distance-weighted
     * averaging in (relX, relY, bearing) space.
     * 
     * Each calibration point stores {relX, relY, bearingDeg, shooterRPM}.
     * 
     * @param relX    Robot X relative to target (meters)
     * @param relY    Robot Y relative to target (meters)
     * @param bearing Robot-relative bearing to target (degrees, -180 to +180)
     * @param table   List of {relX, relY, bearingDeg, shooterRPM}
     * @return double[1]: {shooterRPM}
     */
    private double[] interpolateUnified(double relX, double relY, double bearing, List<double[]> table) {
        if (table == null || table.isEmpty()) return new double[]{3000};
        if (table.size() == 1) {
            double[] pt = table.get(0);
            return new double[]{pt[3]};
        }

        double bearingWeight = Constants.Shooter.CALIBRATION_BEARING_WEIGHT;
        double totalWeight = 0.0;
        double weightedRPM = 0.0;

        for (double[] point : table) {
            double dxp = relX - point[0];
            double dyp = relY - point[1];
            double dBearing = normalizeAngle(bearing - point[2]) * bearingWeight;
            double proximity = Math.sqrt(dxp * dxp + dyp * dyp + dBearing * dBearing);

            if (proximity < 0.001) {
                return new double[]{point[3]};
            }

            double weight = 1.0 / (proximity * proximity);
            totalWeight += weight;
            weightedRPM += weight * point[3];
        }

        return new double[]{weightedRPM / totalWeight};
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

    /** Whether robot is moving fast enough for lead compensation. */
    public boolean isMoving() { return isMoving; }

    /** Whether currently aiming at trench vs hub. */
    public boolean isAimingAtTrench() { return aimingAtTrench; }

    /** Robot-relative bearing to target before offsets (degrees, -180 to +180). */
    public double getRawBearing() { return rawBearing; }

    /** Robot X position relative to target (meters). Alliance-independent. */
    public double getRelativeX() { return relativeX; }

    /** Robot Y position relative to target (meters). Alliance-independent. */
    public double getRelativeY() { return relativeY; }

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
            double fieldX = targetPos.getX() + pt[0];
            double fieldY = targetPos.getY() + pt[1];
            fieldPoses[i] = new Pose2d(fieldX, fieldY, Rotation2d.fromDegrees(pt[2]));
        }
        calibrationField.getObject("CalPoints").setPoses(fieldPoses);
        lastPublishedTarget = targetPos;
        calibrationPointsNeedUpdate = false;
    }
}
