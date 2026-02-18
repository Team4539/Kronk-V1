package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Elastic;

/**
 * Calibration command for fine-tuning shooter power offsets at various distances.
 * 
 * After initial calibration, you might find that shots consistently:
 * - Fall short -> Increase bottom motor power
 * - Go too high -> Decrease top motor power
 * - Miss left/right -> Adjust turret angle offset in CalibrationManager
 * 
 * This command lets you manually set shooter power via CalibrationManager sliders
 * while observing the distance readout, so you can find the right adjustments.
 * 
 * HOW TO USE:
 * 
 * 1. Run this command with the robot at a known distance
 * 2. Adjust Tuning/Shooter/TopPower and BottomPower sliders
 * 3. Shoot and observe results
 * 4. Repeat at multiple distances to verify
 * 5. Record good values via CalibrationManager's RecordPoint button
 * 
 * TIPS:
 * 
 * - Start with small changes (+/-0.02 to +/-0.05)
 * - Test at multiple distances before committing
 * - If different distances need very different corrections, the calibration
 *   table itself needs updating (use FullShooterCalibrationCommand)
 */
public class DistanceOffsetCalibrationCommand extends Command {
    
    // SUBSYSTEMS
    
    private final ShooterSubsystem shooter;
    private final LimelightSubsystem limelight;
    
    // CALIBRATION MANAGER
    
    private final CalibrationManager calibration;
    
    // CONSTRUCTOR
    
    /**
     * Creates a distance offset calibration command.
     * 
     * @param shooter The shooter subsystem  
     * @param limelight The limelight subsystem
     */
    public DistanceOffsetCalibrationCommand(
            ShooterSubsystem shooter,
            LimelightSubsystem limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        this.calibration = CalibrationManager.getInstance();
        
        addRequirements(shooter);
    }
    
    // COMMAND LIFECYCLE
    
    @Override
    public void initialize() {
        // Start with interpolated power (not manual)
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", false);
        
        SmartDashboard.putString("Cal/Status", "OFFSET CALIBRATION - Tune global power offsets");
        SmartDashboard.putString("Cal/Instructions", 
            "1. Position at various distances. " +
            "2. Shooter uses interpolated power. " +
            "3. Adjust Cal/Shooter/Top/BottomOffset. " +
            "4. Test at multiple distances.");
        
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Offset Calibration Started",
            "Adjust global offsets that apply to all distances"));
    }
    
    @Override
    public void execute() {
        // ----- Update distance from limelight -----
        double distance = 3.0; // Default if no vision
        if (limelight.hasPoseEstimate()) {
            distance = limelight.getDistanceToHub();
            calibration.setCurrentDistance(distance);
        }
        
        // ----- Apply manual power from calibration sliders -----
        double topPower = calibration.getShooterTopPower();
        double bottomPower = calibration.getShooterBottomPower();
        shooter.setManualPower(topPower, bottomPower);
        
        // ----- Update status display -----
        String status;
        if (!limelight.hasPoseEstimate()) {
            status = String.format("WARNING: No pose | Top=%.2f Bot=%.2f", topPower, bottomPower);
        } else {
            status = String.format("OK Dist: %.2fm | Top=%.2f Bot=%.2f | %s",
                distance, topPower, bottomPower,
                shooter.isReady() ? "READY" : "Spinning...");
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
            String.format("Final Top=%.3f Bot=%.3f", calibration.getShooterTopPower(), calibration.getShooterBottomPower())));
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until cancelled
    }
}
