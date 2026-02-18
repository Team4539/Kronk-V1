package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Elastic;

/**
 * Interactive command for building the shooter power calibration table.
 * Spins the shooter at manual power from CalibrationManager sliders
 * so you can test shots and record good distance/power pairs.
 * 
 * HOW TO USE:
 * 1. Run this command from SmartDashboard ("Tuning/Cal: Shooting")
 * 2. Adjust Tuning/Shooter/TopPower and BottomPower sliders
 * 3. Shooter spins at the set power so you can shoot
 * 4. Click Tuning/RecordPoint to log the calibration point
 * 5. Move to new distance and repeat
 * 6. Click Tuning/PrintTable to export ready-to-paste Java code
 * 7. Cancel the command to stop
 */
public class ShootingCalibrationCommand extends Command {
    
    // Subsystems
    
    private final ShooterSubsystem shooter;
    private final LimelightSubsystem limelight;
    
    // Calibration Manager
    
    private final CalibrationManager calibration;
    
    // Constructor

    /**
     * Creates a new shooter calibration command.
     * 
     * @param shooter   The shooter subsystem to control
     * @param limelight The limelight for distance measurement
     */
    public ShootingCalibrationCommand(ShooterSubsystem shooter, LimelightSubsystem limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        this.calibration = CalibrationManager.getInstance();
        addRequirements(shooter);
    }

    // COMMAND LIFECYCLE

    @Override
    public void initialize() {
        // Set calibration mode to manual
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", true);
        
        // Display instructions
        SmartDashboard.putString("Cal/Status", "SHOOTING CALIBRATION - Adjust powers and shoot!");
        SmartDashboard.putString("Cal/Instructions", 
            "1. Adjust Cal/Shooter/TopPower (height) and BottomPower (distance). " +
            "2. Click Cal/RecordShootingPoint after a good shot. " +
            "3. Click Cal/PrintShooterTable when done.");
        
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Shooting Calibration Started",
            "Adjust power sliders, record good shots."));
    }

    @Override
    public void execute() {
        // ----- Apply power from CalibrationManager (reads Cal/Shooter/ sliders) -----
        double topPower = calibration.getShooterTopPower();
        double bottomPower = calibration.getShooterBottomPower();
        shooter.setManualPower(topPower, bottomPower);
        
        // ----- Update CalibrationManager with sensor data -----
        if (limelight.hasPoseEstimate()) {
            calibration.setCurrentDistance(limelight.getDistanceToHub());
        }
        
        // ----- Update status display -----
        double distance = limelight.hasPoseEstimate() ? limelight.getDistanceToHub() : -1;
        SmartDashboard.putBoolean("Cal/Shooter/ShooterReady", shooter.isReady());
        
        if (distance < 0) {
            SmartDashboard.putString("Cal/Status", "WARNING: No pose - distance unknown");
        } else {
            SmartDashboard.putString("Cal/Status", 
                String.format("Distance: %.2fm | Top: %.2f | Bottom: %.2f | %s", 
                    distance, topPower, bottomPower,
                    shooter.isReady() ? "READY" : "Spinning..."));
        }
    }

    // CLEANUP

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", false);
        
        int points = calibration.getRecordCount();
        SmartDashboard.putString("Cal/Status", "Shooting calibration ended - " + points + " points recorded");
        
        if (points > 0) {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.INFO, "Shooting Calibration Complete",
                points + " points recorded. Click Cal/PrintShooterTable to export."));
        } else {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.WARNING, "Shooting Calibration Ended",
                "No points recorded."));
        }
    }

    @Override
    public boolean isFinished() {
        // Runs until manually canceled via dashboard button
        return false;
    }
}
