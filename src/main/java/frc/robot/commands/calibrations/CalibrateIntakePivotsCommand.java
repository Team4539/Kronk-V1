package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Calibrates the intake pivot CANcoder offset by setting the current position as "zero".
 * 
 * HOW TO USE:
 * 1. Manually move the intake to the fully retracted (up) position
 * 2. Run this command
 * 3. The current CANcoder reading will be displayed
 * 4. Copy the offset value to Constants.java
 * 
 * The offset shown is the NEGATIVE of the current reading, so when applied,
 * the current position will read as 0 degrees.
 */
public class CalibrateIntakePivotsCommand extends Command {
    
    private final IntakeSubsystem intake;
    
    /**
     * Creates the intake pivot calibration command.
     * 
     * @param intake The intake subsystem
     */
    public CalibrateIntakePivotsCommand(IntakeSubsystem intake) {
        this.intake = intake;
        // Don't require intake - we're just reading values
    }
    
    @Override
    public void initialize() {
        System.out.println("================================================================");
        System.out.println("         INTAKE PIVOT CANCODER CALIBRATION                      ");
        System.out.println("================================================================");
        System.out.println(" 1. Manually move intake to FULLY RETRACTED position            ");
        System.out.println(" 2. Press the button again to capture values                    ");
        System.out.println("================================================================");
        
        SmartDashboard.putString("Calibration/IntakeStatus", "MOVE INTAKE TO RETRACTED POSITION, then press button again");
    }
    
    @Override
    public void execute() {
        // Read current CANcoder value
        double currentAngle = intake.getCurrentPivotAngle();
        
        // Display current reading
        SmartDashboard.putNumber("Calibration/Intake/CurrentAngle", currentAngle);
        
        // Calculate offset (negative of current reading = new zero)
        double offset = -currentAngle;
        
        SmartDashboard.putNumber("Calibration/Intake/Offset", offset);
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // User pressed button again - capture values!
            double currentAngle = intake.getCurrentPivotAngle();
            double offset = -currentAngle;
            
            System.out.println("");
            System.out.println("================================================================");
            System.out.println("         CALIBRATION VALUES CAPTURED!                           ");
            System.out.println("================================================================");
            System.out.println("                                                                ");
            System.out.println("  Copy this value to Constants.java -> Intake class:           ");
            System.out.println("                                                                ");
            System.out.printf("  RIGHT_CANCODER_OFFSET_DEG = %8.3f                          %n", offset);
            System.out.println("                                                                ");
            System.out.println("================================================================");
            System.out.println("");
            
            SmartDashboard.putString("Calibration/IntakeStatus", "DONE! Copy values from console");
            SmartDashboard.putNumber("Calibration/Intake/FINAL_Offset", offset);
        }
    }
    
    @Override
    public boolean isFinished() {
        // Never finishes on its own - user must cancel/re-run to capture
        return false;
    }
}
