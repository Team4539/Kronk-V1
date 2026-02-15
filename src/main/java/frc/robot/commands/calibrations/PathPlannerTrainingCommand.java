package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.ShootingTrainingManager;
import frc.robot.util.ShootingTrainingManager.ShotResult;
import frc.robot.util.ShootingTrainingManager.TargetType;

/**
 * Training command designed to work with PathPlanner event markers.
 * 
 * When a PathPlanner auto reaches a "TrainingShot" event marker, this command
 * runs: it stops the robot, aims the turret, spins the shooter, and waits for
 * the operator to adjust settings via SmartDashboard and record a training point.
 * 
 * ALL ADJUSTMENTS ARE VIA SMARTDASHBOARD ONLY - NO CONTROLLER INPUT!
 * This ensures repeatable, precise training data collection.
 * 
 * WORKFLOW:
 *   1. Build a PathPlanner auto with "TrainingShot" event markers at each position
 *   2. Run the auto - robot drives to first position
 *   3. At each marker, robot stops and this command runs
 *   4. Adjust on SmartDashboard:
 *      - Training/AngleCorrection: turret angle offset (degrees, + = right)
 *      - Training/TopPowerCorrection: top motor power offset
 *      - Training/BottomPowerCorrection: bottom motor power offset
 *      - Training/ShotResult: HIT, SHORT, LONG, LEFT, RIGHT
 *   5. Toggle Training/RecordShot to save the data point
 *   6. Toggle Training/ContinuePath to move to next position
 *   7. Repeat until path is done
 *   8. Copy lines from ShootingTraining/CopyThisLine into CSV
 * 
 * The command automatically captures:
 *   - Distance to target (from Limelight pose)
 *   - Robot pose (X, Y, heading)
 *   - Turret angle
 *   - Battery voltage
 *   - All corrections from SmartDashboard
 */
public class PathPlannerTrainingCommand extends Command {

    // Subsystems
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final LimelightSubsystem limelight;

    // Training manager
    private final ShootingTrainingManager trainingManager;

    // State
    private double angleCorrection = 0.0;
    private double topPowerCorrection = 0.0;
    private double bottomPowerCorrection = 0.0;
    private double baseAimAngle = 0.0;
    private int shotCount = 0;
    private boolean finished = false;

    // Dashboard keys (all under "Training/" prefix)
    private static final String KEY_ANGLE_CORRECTION = "Training/AngleCorrection";
    private static final String KEY_TOP_POWER_CORRECTION = "Training/TopPowerCorrection";
    private static final String KEY_BOTTOM_POWER_CORRECTION = "Training/BottomPowerCorrection";
    private static final String KEY_SHOT_RESULT = "Training/ShotResult";
    private static final String KEY_RECORD_SHOT = "Training/RecordShot";
    private static final String KEY_CONTINUE_PATH = "Training/ContinuePath";
    private static final String KEY_STATUS = "Training/Status";
    private static final String KEY_TARGET_TYPE = "Training/TargetType";
    private static final String KEY_DISTANCE = "Training/CurrentDistance";
    private static final String KEY_TURRET_ANGLE = "Training/CurrentTurretAngle";
    private static final String KEY_SHOT_COUNT = "Training/ShotCount";
    private static final String KEY_ROBOT_X = "Training/RobotX";
    private static final String KEY_ROBOT_Y = "Training/RobotY";
    private static final String KEY_ROBOT_HEADING = "Training/RobotHeading";
    private static final String KEY_VOLTAGE = "Training/BatteryVoltage";

    /**
     * Creates a PathPlanner training command.
     * This is registered as a named command so PathPlanner can trigger it.
     *
     * @param turret    Turret subsystem for aiming
     * @param shooter   Shooter subsystem for power control
     * @param limelight Limelight subsystem for distance/pose
     */
    public PathPlannerTrainingCommand(TurretSubsystem turret, ShooterSubsystem shooter,
                                      LimelightSubsystem limelight) {
        this.turret = turret;
        this.shooter = shooter;
        this.limelight = limelight;
        this.trainingManager = ShootingTrainingManager.getInstance();

        addRequirements(turret, shooter);
    }

    @Override
    public void initialize() {
        finished = false;
        shotCount = 0;

        // Reset dashboard controls to defaults
        SmartDashboard.putNumber(KEY_ANGLE_CORRECTION, 0.0);
        SmartDashboard.putNumber(KEY_TOP_POWER_CORRECTION, 0.0);
        SmartDashboard.putNumber(KEY_BOTTOM_POWER_CORRECTION, 0.0);
        SmartDashboard.putString(KEY_SHOT_RESULT, "HIT");
        SmartDashboard.putString(KEY_TARGET_TYPE, "HUB");
        SmartDashboard.putBoolean(KEY_RECORD_SHOT, false);
        SmartDashboard.putBoolean(KEY_CONTINUE_PATH, false);

        // Capture the initial auto-aim angle as baseline
        baseAimAngle = turret.getTargetAngle();

        // Update status
        updateSensorDisplay();
        SmartDashboard.putString(KEY_STATUS, "TRAINING STOP - Adjust via SmartDashboard, then RecordShot");

        Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle("🎯 Training Stop")
                .withDescription("Adjust aim & power on SmartDashboard. Toggle RecordShot when ready.")
                .withDisplaySeconds(5.0));

        System.out.println("\n=== PATH PLANNER TRAINING STOP ===");
        System.out.println("Adjust settings on SmartDashboard (NO controller!)");
        System.out.println("Toggle Training/RecordShot to save a point");
        System.out.println("Toggle Training/ContinuePath when done at this spot");
        System.out.println("==================================\n");
    }

    @Override
    public void execute() {
        // Read corrections from SmartDashboard
        angleCorrection = SmartDashboard.getNumber(KEY_ANGLE_CORRECTION, 0.0);
        topPowerCorrection = SmartDashboard.getNumber(KEY_TOP_POWER_CORRECTION, 0.0);
        bottomPowerCorrection = SmartDashboard.getNumber(KEY_BOTTOM_POWER_CORRECTION, 0.0);

        // Apply turret angle with correction
        double targetAngle = baseAimAngle + angleCorrection;
        turret.setTargetAngle(targetAngle);

        // Spin up shooter based on distance with corrections applied
        if (limelight.hasPoseEstimate()) {
            String targetTypeStr = SmartDashboard.getString(KEY_TARGET_TYPE, "HUB");
            double distance;
            if (targetTypeStr.equalsIgnoreCase("TRENCH")) {
                distance = limelight.getDistanceToTrench();
                shooter.setForDistanceTrenchWithOffset(distance, topPowerCorrection, bottomPowerCorrection);
            } else {
                distance = limelight.getDistanceToHub();
                shooter.setForDistanceWithOffset(distance, topPowerCorrection, bottomPowerCorrection);
            }
        }

        // Check for record request
        if (SmartDashboard.getBoolean(KEY_RECORD_SHOT, false)) {
            SmartDashboard.putBoolean(KEY_RECORD_SHOT, false); // Reset toggle
            recordTrainingPoint();
        }

        // Check for continue request
        if (SmartDashboard.getBoolean(KEY_CONTINUE_PATH, false)) {
            SmartDashboard.putBoolean(KEY_CONTINUE_PATH, false); // Reset toggle
            finished = true;
        }

        // Update live sensor display
        updateSensorDisplay();
    }

    /**
     * Records the current state as a training data point.
     * All data comes from sensors + SmartDashboard corrections.
     */
    private void recordTrainingPoint() {
        if (!limelight.hasPoseEstimate()) {
            SmartDashboard.putString(KEY_STATUS, "ERROR: No pose estimate! Cannot record.");
            Elastic.sendNotification(new Elastic.Notification()
                    .withLevel(NotificationLevel.ERROR)
                    .withTitle("❌ Record Failed")
                    .withDescription("No pose estimate available")
                    .withDisplaySeconds(3.0));
            return;
        }

        // Get target type
        String targetTypeStr = SmartDashboard.getString(KEY_TARGET_TYPE, "HUB");
        TargetType targetType = TargetType.HUB;
        try {
            targetType = TargetType.valueOf(targetTypeStr.toUpperCase());
        } catch (Exception e) { /* default HUB */ }

        // Get shot result
        String resultStr = SmartDashboard.getString(KEY_SHOT_RESULT, "HIT");
        ShotResult result = ShotResult.HIT;
        try {
            result = ShotResult.valueOf(resultStr.toUpperCase());
        } catch (Exception e) { /* default HIT */ }

        // Gather sensor data
        double distance = (targetType == TargetType.TRENCH)
                ? limelight.getDistanceToTrench()
                : limelight.getDistanceToHub();
        double robotX = limelight.getRobotPose().getX();
        double robotY = limelight.getRobotPose().getY();
        double robotHeading = limelight.getRobotPose().getRotation().getDegrees();
        double turretAngle = turret.getCurrentAngle();
        double angleToTarget = (targetType == TargetType.TRENCH)
                ? limelight.getAngleToTrench()
                : limelight.getAngleToHub();
        double voltage = RobotController.getBatteryVoltage();

        // Record via training manager (stationary - driven by PathPlanner, stopped at marker)
        trainingManager.recordTrainingPoint(
                targetType, distance,
                robotX, robotY, robotHeading,
                0.0, 0.0, 0.0,  // Stationary at waypoint
                turretAngle, angleToTarget,
                angleCorrection, topPowerCorrection, bottomPowerCorrection,
                voltage, result);

        shotCount++;
        SmartDashboard.putNumber(KEY_SHOT_COUNT, shotCount);

        String msg = String.format("Recorded #%d: %s @ %.1fm, angle %+.1f°, top %+.3f, bot %+.3f → %s",
                shotCount, targetType.name(), distance, angleCorrection,
                topPowerCorrection, bottomPowerCorrection, result.name());
        SmartDashboard.putString(KEY_STATUS, msg);

        Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle("✅ Shot #" + shotCount + " Recorded")
                .withDescription(String.format("%.1fm | %+.1f° | %s", distance, angleCorrection, result.name()))
                .withDisplaySeconds(2.0));
    }

    /**
     * Updates the live sensor readout on SmartDashboard.
     */
    private void updateSensorDisplay() {
        if (limelight.hasPoseEstimate()) {
            String targetTypeStr = SmartDashboard.getString(KEY_TARGET_TYPE, "HUB");
            double distance;
            double angleToTarget;
            if (targetTypeStr.equalsIgnoreCase("TRENCH")) {
                distance = limelight.getDistanceToTrench();
                angleToTarget = limelight.getAngleToTrench();
            } else {
                distance = limelight.getDistanceToHub();
                angleToTarget = limelight.getAngleToHub();
            }
            SmartDashboard.putNumber(KEY_DISTANCE, distance);
            SmartDashboard.putNumber(KEY_ROBOT_X, limelight.getRobotPose().getX());
            SmartDashboard.putNumber(KEY_ROBOT_Y, limelight.getRobotPose().getY());
            SmartDashboard.putNumber(KEY_ROBOT_HEADING, limelight.getRobotPose().getRotation().getDegrees());
            SmartDashboard.putNumber("Training/AngleToTarget", angleToTarget);
        } else {
            SmartDashboard.putNumber(KEY_DISTANCE, -1);
        }
        SmartDashboard.putNumber(KEY_TURRET_ANGLE, turret.getCurrentAngle());
        SmartDashboard.putNumber(KEY_VOLTAGE, RobotController.getBatteryVoltage());
        SmartDashboard.putBoolean("Training/ShooterReady", shooter.isReady());
        SmartDashboard.putBoolean("Training/HasPose", limelight.hasPoseEstimate());
        SmartDashboard.putNumber(KEY_SHOT_COUNT, shotCount);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();

        if (interrupted) {
            SmartDashboard.putString(KEY_STATUS, "Training interrupted - " + shotCount + " shots recorded");
        } else {
            SmartDashboard.putString(KEY_STATUS, "Training stop complete - " + shotCount + " shots, continuing path");
        }

        Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle("Training Stop Done")
                .withDescription(shotCount + " shots recorded. Copy CSV lines from ShootingTraining/CopyThisLine!")
                .withDisplaySeconds(3.0));

        System.out.println("=== TRAINING STOP COMPLETE: " + shotCount + " shots recorded ===");
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
