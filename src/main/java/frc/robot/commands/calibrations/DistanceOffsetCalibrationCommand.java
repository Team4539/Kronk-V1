package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Elastic;

public class DistanceOffsetCalibrationCommand extends Command {
    
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final CalibrationManager calibration;
    
    public DistanceOffsetCalibrationCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        this.calibration = CalibrationManager.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", false);
        SmartDashboard.putString("Cal/Status", "OFFSET CALIBRATION - Tune global RPM offset");
        SmartDashboard.putString("Cal/Instructions", 
            "1. Position at various distances. " +
            "2. Adjust Tuning/Shooter/RPM. " +
            "3. Test at multiple distances.");
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Offset Calibration Started",
            "Adjust global offsets that apply to all distances"));
    }
    
    @Override
    public void execute() {
        double distance = 3.0;
        if (vision.hasPoseEstimate()) {
            distance = vision.getDistanceToHub();
            calibration.setCurrentDistance(distance);
        }
        double rpm = calibration.getShooterRPM();
        shooter.setTargetRPM(rpm);
        String status;
        if (!vision.hasPoseEstimate()) {
            status = String.format("WARNING: No pose | RPM=%.0f", rpm);
        } else {
            status = String.format("OK Dist: %.2fm | RPM=%.0f | %s",
                distance, rpm, shooter.isReady() ? "READY" : "Spinning...");
        }
        SmartDashboard.putString("Cal/Status", status);
        SmartDashboard.putNumber("Cal/Offset/CurrentDistance", distance);
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        SmartDashboard.putString("Cal/Status", "Offset calibration ended");
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Offset Calibration Ended",
            String.format("Final RPM=%.0f", calibration.getShooterRPM())));
    }
    
    @Override
    public boolean isFinished() { return false; }
}
