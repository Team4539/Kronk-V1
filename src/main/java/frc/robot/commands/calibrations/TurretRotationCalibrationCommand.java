package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.ShootingCalculator;

/**
 * Calibration command for building the turret rotation offset table.
 * 
 * At different distances the turret may need a different angle correction
 * to hit the target. This command lets you drive to various distances,
 * adjust the Tuning/Turret/RotationOffset slider until shots are on-target,
 * then record the distance-offset pair. When done, print the table and
 * paste it into Constants.Turret.ROTATION_CALIBRATION.
 * 
 * The turret aims using ShootingCalculator (auto-tracking the target) while
 * applying the live RotationOffset on top, so you can see the effect in
 * real-time.
 * 
 * HOW TO USE:
 * 1. Run this command from SmartDashboard ("Tuning/Cal: Turret Rotation")
 * 2. Drive to a known distance from the target
 * 3. Adjust Tuning/Turret/RotationOffset slider until turret is aimed correctly
 * 4. Click Tuning/RecordRotationPoint to save the distance+offset pair
 * 5. Move to a new distance and repeat
 * 6. Click Tuning/PrintRotationTable to get copy-paste code for Constants.java
 * 7. Cancel the command when done
 */
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
                "1. Drive to a distance. " +
                "2. Adjust Tuning/Turret/RotationOffset until turret aims correctly. " +
                "3. Click Tuning/RecordRotationPoint. " +
                "4. Repeat at more distances. " +
                "5. Click Tuning/PrintRotationTable when done.");

        Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.INFO, "Turret Rotation Calibration Started",
                "Adjust rotation offset at various distances."));
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

        // Status display
        double distance = calibration.getCurrentDistance();
        double rotationOffset = calibration.getTurretRotationOffset();

        String status;
        if (!vision.hasPoseEstimate()) {
            status = String.format("WARNING: No pose | RotOffset: %.2f°", rotationOffset);
        } else {
            status = String.format("Dist: %.2fm | RotOffset: %.2f° | TurretCmd: %.1f°",
                    distance, rotationOffset, angle);
        }

        SmartDashboard.putString("Cal/Status", status);
        SmartDashboard.putNumber("Cal/Rotation/Distance", distance);
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
