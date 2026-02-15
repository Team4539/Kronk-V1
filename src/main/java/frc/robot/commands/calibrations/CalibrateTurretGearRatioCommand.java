package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Determines the correct turret gear ratio by measuring motor rotations per full turret rotation.
 * Run command, manually rotate turret exactly 360 degrees, then check SmartDashboard for calculated ratio.
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
        SmartDashboard.putNumber("Calibration/StartMotorRotations", startMotorRotations);
        SmartDashboard.putString("Calibration/Status", "CALIBRATING - Rotate turret 360 degrees by hand, then end command");
        SmartDashboard.putNumber("Calibration/CalculatedGearRatio", 0.0);
        SmartDashboard.putNumber("Calibration/MotorRotationsFor360", 0.0);
        
        // Print instructions to console
        System.out.println("=== TURRET GEAR RATIO CALIBRATION STARTED ===");
        System.out.println("Start motor position: " + startMotorRotations + " rotations");
        System.out.println("Now rotate the turret EXACTLY 360 degrees by hand.");
        System.out.println("Then press the button again or cancel the command.");
    }
    
    @Override
    public void execute() {
        // Continuously update readings while calibrating
        double currentMotorRotations = turret.getMotorRotations();
        double motorRotationsDelta = Math.abs(currentMotorRotations - startMotorRotations);
        
        SmartDashboard.putNumber("Calibration/CurrentMotorRotations", currentMotorRotations);
        SmartDashboard.putNumber("Calibration/MotorRotationsSoFar", motorRotationsDelta);
        
        // Show live preview of gear ratio (assuming current position is 360 degrees)
        if (motorRotationsDelta > 0.1) {
            double previewGearRatio = motorRotationsDelta;  // motor rotations per 1 turret rotation
            SmartDashboard.putNumber("Calibration/PreviewGearRatio", previewGearRatio);
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
        SmartDashboard.putNumber("Calibration/EndMotorRotations", endMotorRotations);
        SmartDashboard.putNumber("Calibration/MotorRotationsFor360", motorRotationsFor360);
        SmartDashboard.putNumber("Calibration/CalculatedGearRatio", calculatedGearRatio);
        SmartDashboard.putString("Calibration/Status", "DONE - Copy CalculatedGearRatio to Constants.java");
        
        // Print results to console
        System.out.println("=== TURRET GEAR RATIO CALIBRATION COMPLETE ===");
        System.out.println("End motor position: " + endMotorRotations + " rotations");
        System.out.println("Motor rotations for 360 degrees: " + motorRotationsFor360);
        System.out.println("");
        System.out.println(">>> CALCULATED GEAR RATIO: " + calculatedGearRatio + " <<<");
        System.out.println("");
        System.out.println("Copy this value to Constants.Turret.GEAR_RATIO");
        
        calibrationStarted = false;
    }
    
    @Override
    public boolean isFinished() {
        // Runs until manually canceled
        return false;
    }
}
