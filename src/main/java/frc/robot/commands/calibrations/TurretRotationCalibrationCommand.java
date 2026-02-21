package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.ShootingCalculator;

/**
 * @deprecated Use FullShooterCalibrationCommand instead — calibration is now unified.
 * This command is kept for backwards compatibility but records to the same
 * unified calibration table (x, y, bearing, turretOffset, topRPM, bottomRPM).
 * 
 * The turret angle offset depends on BOTH distance to target AND the robot's
 * orientation relative to the target (bearing). This command lets you drive 
 * to various positions, adjust the RotationOffset slider, and record unified
 * calibration points.
 * 
 * HOW TO USE:
 * 1. Run this command from SmartDashboard ("Tuning/Cal: Turret Rotation")
 * 2. Drive to a position — note BOTH the distance AND bearing on the dashboard
 * 3. Adjust Tuning/Turret/RotationOffset slider until turret is aimed correctly
 * 4. Click Tuning/RecordPoint to save the unified calibration point
 * 5. Move to a new distance OR turn the robot to a different orientation and repeat
 * 6. Click Tuning/PrintTable to get copy-paste code for Constants.java
 * 7. Cancel the command when done
 */
@Deprecated
public class TurretRotationCalibrationCommand extends Command {

    private final TurretSubsystem turret;
    private final VisionSubsystem vision;
    private final CalibrationManager calibration;
    private final ShootingCalculator shootingCalc;

    /**
     * Creates a turret rotation offset calibration command.
     *
     * @param turret The turret subsystem (for angle control)
     * @param vision The vision subsystem (for distance measurement)
     */
    public TurretRotationCalibrationCommand(TurretSubsystem turret, VisionSubsystem vision) {
        this.turret = turret;
        this.vision = vision;
        this.calibration = CalibrationManager.getInstance();
        this.shootingCalc = ShootingCalculator.getInstance();
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Cal/Status",
                "ROTATION CALIBRATION - Adjust RotationOffset slider, record at each distance");
        SmartDashboard.putString("Cal/Instructions",
                "1. Drive to a distance & orientation. " +
                "2. Adjust Tuning/Turret/RotationOffset until turret aims correctly. " +
                "3. Click Tuning/RecordRotationPoint (records distance + bearing + offset). " +
                "4. Repeat at different distances AND orientations. " +
                "5. Click Tuning/PrintRotationTable when done.");

        Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.INFO, "Turret Rotation Calibration Started",
                "Adjust rotation offset at various distances and orientations."));
    }

    @Override
    public void execute() {
        // Turret auto-tracks via ShootingCalculator (which already includes
        // the live RotationOffset via the normal update loop in RobotContainer).
        // We just need to command the turret to the calculated angle.
        double angle = shootingCalc.getTurretAngle();
        turret.setTargetAngle(angle);

        // Update CalibrationManager with sensor distance
        if (vision.hasPoseEstimate()) {
            calibration.setCurrentDistance(vision.getDistanceToHub());
        }

        // Status display — now shows bearing (robot orientation relative to target)
        double distance = calibration.getCurrentDistance();
        double bearing = shootingCalc.getRawBearing();
        double rotationOffset = calibration.getTurretRotationOffset();

        String status;
        if (!vision.hasPoseEstimate()) {
            status = String.format("WARNING: No pose | RotOffset: %.2f°", rotationOffset);
        } else {
            status = String.format("Dist: %.2fm | Bearing: %.0f° | RotOffset: %.2f° | TurretCmd: %.1f°",
                    distance, bearing, rotationOffset, angle);
        }

        SmartDashboard.putString("Cal/Status", status);
        SmartDashboard.putNumber("Cal/Rotation/Distance", distance);
        SmartDashboard.putNumber("Cal/Rotation/Bearing", bearing);
        SmartDashboard.putNumber("Cal/Rotation/Offset", rotationOffset);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Cal/Status", "Rotation calibration ended");

        Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.INFO, "Turret Rotation Calibration Ended",
                "Click PrintRotationTable to export recorded offsets."));
    }

    @Override
    public boolean isFinished() {
        return false; // Run until cancelled
    }
}
