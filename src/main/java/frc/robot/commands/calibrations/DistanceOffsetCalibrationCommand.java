package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * DISTANCE OFFSET CALIBRATION COMMAND
 * Calibration command for fine-tuning distance-based offsets.
 * 
 * This command lets you observe how the shooter performs at different distances
 * and record global offsets that apply to the entire calibration table.
 * 
 * USE CASE:
 * 
 * After initial calibration, you might find that shots consistently:
 * - Fall short - Increase bottom motor offset
 * - Go too high - Decrease top motor offset
 * - Miss left/right - Adjust turret angle offset
 * 
 * Rather than re-calibrate the entire table, you can add small global offsets.
 * 
 * HOW TO USE:
 * 
 * 1. Run this command with the robot at a known distance
 * 2. Use distance-based (interpolated) power by setting UseManual = false
 * 3. Shoot and observe results
 * 4. Adjust Cal/Shooter/TopOffset and Cal/Shooter/BottomOffset
 * 5. Shoot again until shots are consistently good
 * 6. Repeat at multiple distances to verify offsets work everywhere
 * 7. Click "Cal/PrintAllConstants" to get the offset values
 * 
 * TIPS:
 * 
 * - Start with small offsets (+/-0.02 to +/-0.05)
 * - Positive offset = more power
 * - Test at multiple distances before committing to an offset
 * - If different distances need different corrections, the calibration
 *   table itself needs adjustment (use FullShooterCalibrationCommand)
 * 
 * @author Team 4539
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
        
        System.out.println("=== DISTANCE OFFSET CALIBRATION STARTED ===");
        System.out.println("Adjust global offsets that apply to all distances");
    }
    
    @Override
    public void execute() {
        // ----- Update distance from limelight -----
        double distance = 3.0; // Default if no vision
        if (limelight.hasPoseEstimate()) {
            distance = limelight.getDistanceToHub();
            calibration.setCurrentDistance(distance);
        }
        
        // Update target info
        calibration.setHasTarget(limelight.hasTarget());
        if (limelight.hasTarget()) {
            calibration.setCurrentTagId(limelight.getTargetId());
        }
        
        // ----- Apply power based on mode -----
        if (calibration.useManualShooterPower()) {
            // Manual mode - direct power control
            shooter.setManualPower(
                calibration.getShooterTopPower(),
                calibration.getShooterBottomPower()
            );
        } else {
            // Interpolated mode - distance-based with offsets
            shooter.setForDistanceWithOffset(
                distance,
                calibration.getShooterTopOffset(),
                calibration.getShooterBottomOffset()
            );
        }
        
        // ----- Update status display -----
        double topOffset = calibration.getShooterTopOffset();
        double bottomOffset = calibration.getShooterBottomOffset();
        boolean useManual = calibration.useManualShooterPower();
        
        String mode = useManual ? "MANUAL" : "INTERPOLATED";
        String status;
        
        if (!limelight.hasPoseEstimate()) {
            status = String.format("WARNING: No pose | Mode: %s | Offsets: Top=%.3f Bot=%.3f",
                mode, topOffset, bottomOffset);
        } else {
            status = String.format("OK Dist: %.2fm | Mode: %s | Offsets: Top=%.3f Bot=%.3f",
                distance, mode, topOffset, bottomOffset);
        }
        
        SmartDashboard.putString("Cal/Status", status);
        
        // Show what power is being applied
        SmartDashboard.putNumber("Cal/Offset/AppliedTopPower", shooter.getCurrentTopPower());
        SmartDashboard.putNumber("Cal/Offset/AppliedBottomPower", shooter.getCurrentBottomPower());
        SmartDashboard.putNumber("Cal/Offset/CurrentDistance", distance);
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        
        SmartDashboard.putString("Cal/Status", "Offset calibration ended");
        
        System.out.println("=== DISTANCE OFFSET CALIBRATION ENDED ===");
        System.out.println("Final offsets:");
        System.out.printf("  Top Motor Offset: %.3f%n", calibration.getShooterTopOffset());
        System.out.printf("  Bottom Motor Offset: %.3f%n", calibration.getShooterBottomOffset());
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until cancelled
    }
}
