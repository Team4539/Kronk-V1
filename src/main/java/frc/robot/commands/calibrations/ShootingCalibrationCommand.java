package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Elastic;

public class ShootingCalibrationCommand extends Command {
    
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final CalibrationManager calibration;

    public ShootingCalibrationCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        this.calibration = CalibrationManager.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", true);
        SmartDashboard.putString("Cal/Status", "SHOOTING CALIBRATION - Adjust RPM and shoot");
        SmartDashboard.putString("Cal/Instructions", 
            "1. Adjust Tuning/Shooter/RPM. " +
            "2. Click Tuning/RecordPoint after a good shot. " +
            "3. Click Tuning/PrintTable when done.");
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Shooting Calibration Started",
            "Adjust RPM slider, record good shots."));
    }

    @Override
    public void execute() {
        double rpm = calibration.getShooterRPM();
        shooter.setTargetRPM(rpm);
        if (vision.hasPoseEstimate()) {
            calibration.setCurrentDistance(vision.getDistanceToHub());
        }
        double distance = vision.hasPoseEstimate() ? vision.getDistanceToHub() : -1;
        SmartDashboard.putBoolean("Cal/Shooter/ShooterReady", shooter.isReady());
        if (distance < 0) {
            SmartDashboard.putString("Cal/Status", "WARNING: No pose - distance unknown");
        } else {
            SmartDashboard.putString("Cal/Status", 
                String.format("Distance: %.2fm | RPM: %.0f | %s", 
                    distance, rpm, shooter.isReady() ? "READY" : "Spinning..."));
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", false);
        int points = calibration.getRecordCount();
        SmartDashboard.putString("Cal/Status", "Shooting calibration ended - " + points + " points recorded");
        if (points > 0) {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.INFO, "Shooting Calibration Complete",
                points + " points recorded. Click PrintTable to export."));
        } else {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.WARNING, "Shooting Calibration Ended", "No points recorded."));
        }
    }

    @Override
    public boolean isFinished() { return false; }
}
