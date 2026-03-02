package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.ShootingCalculator;

public class FullShooterCalibrationCommand extends Command {
    
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final CalibrationManager calibration;
    private final ShootingCalculator shootingCalc;
    
    public FullShooterCalibrationCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        this.calibration = CalibrationManager.getInstance();
        this.shootingCalc = ShootingCalculator.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", true);
        SmartDashboard.putString("Cal/Status", "SHOOTER CALIBRATION - Adjust RPM, then record");
        SmartDashboard.putString("Cal/Instructions", 
            "1. Use Tuning/Shooter/RPM to tune shot. " +
            "2. Use POV Down on controller to feed a ball. " +
            "3. Click Tuning/RecordPoint to save.");
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Shooter Calibration Started",
            "Use sliders to tune shooter RPM."));
    }
    
    @Override
    public void execute() {
        if (vision.hasPoseEstimate()) {
            calibration.setCurrentDistance(vision.getDistanceToHub());
        }
        double rpm = calibration.getShooterRPM();
        shooter.setTargetRPM(rpm);
        updateStatusDisplay();
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", false);
        SmartDashboard.putString("Cal/Status", "Calibration ended");
        int points = calibration.getRecordCount();
        if (points > 0) {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.INFO, "Calibration Complete",
                points + " points recorded. Click PrintTable to export."));
        } else {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.WARNING, "Calibration Ended", "No points recorded."));
        }
    }
    
    @Override
    public boolean isFinished() { return false; }
    
    private void updateStatusDisplay() {
        double distance = calibration.getCurrentDistance();
        double rpm = calibration.getShooterRPM();
        double bearing = shootingCalc.getRawBearing();
        double relX = shootingCalc.getRelativeX();
        double relY = shootingCalc.getRelativeY();
        String status;
        if (!vision.hasTarget()) {
            status = "WARNING: NO TARGET - Move to see AprilTags";
        } else if (distance <= 0) {
            status = "WARNING: Tag visible, no pose";
        } else {
            status = String.format("OK Dist: %.2fm | RelX: %.2f RelY: %.2f | Bearing: %.0f deg | RPM: %.0f", 
                distance, relX, relY, bearing, rpm);
        }
        SmartDashboard.putString("Cal/Status", status);
        SmartDashboard.putNumber("Cal/Robot/RelX", relX);
        SmartDashboard.putNumber("Cal/Robot/RelY", relY);
        SmartDashboard.putNumber("Cal/Robot/Bearing", bearing);
        SmartDashboard.putNumber("Cal/Robot/Distance", distance);
        if (vision.hasTarget()) {
            SmartDashboard.putNumber("Cal/RawTX", vision.getHorizontalOffset());
        }
    }
}
