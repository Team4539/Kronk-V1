package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Elastic;

/**
 * Determines the correct turret gear ratio by measuring motor rotations per full turret rotation.
 * All controls are on SmartDashboard -- no controller buttons needed.
 * 
 * HOW TO USE:
 * 1. Click "Cal/TurretGearRatio" on SmartDashboard to start
 * 2. Manually rotate turret exactly 360 degrees by hand
 * 3. Watch "Cal/GearRatio/PreviewGearRatio" update live
 * 4. Click the command button again to stop and get the final ratio
 * 5. Copy CalculatedGearRatio to Constants.Turret.GEAR_RATIO
 */
public class CalibrateTurretGearRatioCommand extends Command {
    
    private final TurretSubsystem turret;
    
    // STATE
    
    /** Motor position when calibration started */
    private double startMotorRotations = 0.0;
    
    /** Whether calibration has begun */
    private boolean calibrationStarted = false;
    
    // CONSTRUCTOR
    
    /**
     * Creates the turret gear ratio calibration command.
     * 
     * @param turret The turret subsystem (not claimed, just for reading position)
     */
    public CalibrateTurretGearRatioCommand(TurretSubsystem turret) {
        this.turret = turret;
        // NOTE: We intentionally don't add requirements
        // This allows the motor to be moved freely by hand during calibration
    }
    
    // COMMAND LIFECYCLE
    
    @Override
    public void initialize() {
        // Record starting position directly from turret subsystem
        startMotorRotations = turret.getMotorRotations();
        calibrationStarted = true;
        
        // Set up dashboard display
        SmartDashboard.putNumber("Cal/GearRatio/StartMotorRotations", startMotorRotations);
        SmartDashboard.putNumber("Cal/GearRatio/CalculatedGearRatio", 0.0);
        SmartDashboard.putNumber("Cal/GearRatio/MotorRotationsFor360", 0.0);
        
        SmartDashboard.putString("Cal/Status", "GEAR RATIO CAL - Rotate turret 360 degrees by hand, then stop command");
        SmartDashboard.putString("Cal/Instructions",
            "1. Rotate turret EXACTLY 360 degrees by hand. " +
            "2. Watch Cal/GearRatio/PreviewGearRatio update live. " +
            "3. Click the command button again to finalize.");
        
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Gear Ratio Calibration Started",
            "Rotate turret 360 deg by hand. Start: " + startMotorRotations + " rotations"));
    }
    
    @Override
    public void execute() {
        // Continuously update readings while calibrating
        double currentMotorRotations = turret.getMotorRotations();
        double motorRotationsDelta = Math.abs(currentMotorRotations - startMotorRotations);
        
        SmartDashboard.putNumber("Cal/GearRatio/CurrentMotorRotations", currentMotorRotations);
        SmartDashboard.putNumber("Cal/GearRatio/MotorRotationsSoFar", motorRotationsDelta);
        
        // Show live preview of gear ratio (assuming current position is 360 degrees)
        if (motorRotationsDelta > 0.1) {
            double previewGearRatio = motorRotationsDelta;  // motor rotations per 1 turret rotation
            SmartDashboard.putNumber("Cal/GearRatio/PreviewGearRatio", previewGearRatio);
            
            SmartDashboard.putString("Cal/Status", 
                String.format("GEAR RATIO CAL - Motor delta: %.2f rot | Preview ratio: %.2f", 
                    motorRotationsDelta, previewGearRatio));
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        if (!calibrationStarted) {
            return;
        }
        
        // Calculate final gear ratio
        double endMotorRotations = turret.getMotorRotations();
        double motorRotationsFor360 = Math.abs(endMotorRotations - startMotorRotations);
        
        // Gear ratio = motor rotations per turret rotation
        // If you rotated the turret 360 degrees (1 rotation), motor rotations = gear ratio
        double calculatedGearRatio = motorRotationsFor360;
        
        // Update dashboard
        SmartDashboard.putNumber("Cal/GearRatio/EndMotorRotations", endMotorRotations);
        SmartDashboard.putNumber("Cal/GearRatio/MotorRotationsFor360", motorRotationsFor360);
        SmartDashboard.putNumber("Cal/GearRatio/CalculatedGearRatio", calculatedGearRatio);
        SmartDashboard.putString("Cal/Status", "DONE - Copy Cal/GearRatio/CalculatedGearRatio to Constants.java");
        
        // Print results to console
        System.out.println("=== TURRET GEAR RATIO CALIBRATION COMPLETE ===");
        System.out.println("Motor rotations for 360 degrees: " + motorRotationsFor360);
        System.out.println(">>> CALCULATED GEAR RATIO: " + calculatedGearRatio + " <<<");
        System.out.println("Copy this value to Constants.Turret.GEAR_RATIO");
        
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Gear Ratio Calculated",
            String.format("Ratio: %.2f (%.2f motor rot for 360 deg)", calculatedGearRatio, motorRotationsFor360)));
        
        calibrationStarted = false;
    }
    
    @Override
    public boolean isFinished() {
        // Runs until manually canceled via dashboard button
        return false;
    }
}
