package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.VisionSubsystem;
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
    private final VisionSubsystem vision;
    
    // CALIBRATION MANAGER
    
    private final CalibrationManager calibration;
    
    // CONSTRUCTOR
    
    /**
     * Creates a distance offset calibration command.
     * 
     * @param shooter The shooter subsystem  
     * @param vision The vision subsystem
     */
    public DistanceOffsetCalibrationCommand(
            ShooterSubsystem shooter,
            VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
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
            "2. Shooter uses manual RPM from sliders. " +
            "3. Adjust Tuning/Shooter/TopRPM and BottomRPM. " +
            "4. Test at multiple distances.");
        
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Offset Calibration Started",
            "Adjust global offsets that apply to all distances"));
    }
    
    @Override
    public void execute() {
        // ----- Update distance from vision -----
        double distance = 3.0; // Default if no vision
        if (vision.hasPoseEstimate()) {
            distance = vision.getDistanceToHub();
            calibration.setCurrentDistance(distance);
        }
        
        // ----- Apply RPM from calibration sliders -----
        double topRPM = calibration.getShooterTopRPM();
        double bottomRPM = calibration.getShooterBottomRPM();
        shooter.setTargetRPM(topRPM, bottomRPM);
        
        // ----- Update status display -----
        String status;
        if (!vision.hasPoseEstimate()) {
            status = String.format("WARNING: No pose | Top=%.0f RPM Bot=%.0f RPM", topRPM, bottomRPM);
        } else {
            status = String.format("OK Dist: %.2fm | Top=%.0f RPM Bot=%.0f RPM | %s",
                distance, topRPM, bottomRPM,
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
            String.format("Final Top=%.0f RPM Bot=%.0f RPM", calibration.getShooterTopRPM(), calibration.getShooterBottomRPM())));
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until cancelled
    }
}
