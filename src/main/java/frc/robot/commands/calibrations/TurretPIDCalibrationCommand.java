package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Live PID tuning command for the turret motor.
 * Adjust P, I, D gains via SmartDashboard and see results in real-time.
 */
public class TurretPIDCalibrationCommand extends Command {
    
    private final TurretSubsystem turret;
    private final CalibrationManager calibration;
    
    /** Target angle for PID testing */
    private double targetAngle = 0.0;
    
    /** Previous P value to detect changes */
    private double lastP = 0.0;
    
    /** Previous I value to detect changes */
    private double lastI = 0.0;
    
    /** Previous D value to detect changes */
    private double lastD = 0.0;
    
    // CONSTRUCTOR
    
    /**
     * Creates a turret PID calibration command.
     * 
     * @param turret The turret subsystem to tune
     */
    public TurretPIDCalibrationCommand(TurretSubsystem turret) {
        this.turret = turret;
        this.calibration = CalibrationManager.getInstance();
        addRequirements(turret);
    }
    
    // COMMAND LIFECYCLE
    
    @Override
    public void initialize() {
        // Initialize target angle slider
        SmartDashboard.putNumber("Cal/PID/TargetAngle", 0.0);
        SmartDashboard.putNumber("Cal/PID/StepSize", 30.0);
        SmartDashboard.putBoolean("Cal/PID/StepPositive", false);
        SmartDashboard.putBoolean("Cal/PID/StepNegative", false);
        
        // Store initial PID values
        lastP = calibration.getTurretP();
        lastI = calibration.getTurretI();
        lastD = calibration.getTurretD();
        
        SmartDashboard.putString("Cal/Status", "PID TUNING MODE - Adjust P, I, D and target angle");
        SmartDashboard.putString("Cal/Instructions", 
            "1. Set target angle or use Step buttons. " +
            "2. Adjust Cal/Turret/P, I, D. " +
            "3. Watch response and iterate.");
        
        // Notify via Elastic
        calibration.logInfo("PID Tuning Started", 
            String.format("Current: P=%.4f I=%.4f D=%.4f", lastP, lastI, lastD));
    }
    
    @Override
    public void execute() {
        // ----- Read target angle -----
        targetAngle = SmartDashboard.getNumber("Cal/PID/TargetAngle", targetAngle);
        
        // ----- Handle step buttons for quick testing -----
        double stepSize = SmartDashboard.getNumber("Cal/PID/StepSize", 30.0);
        
        if (SmartDashboard.getBoolean("Cal/PID/StepPositive", false)) {
            targetAngle += stepSize;
            SmartDashboard.putNumber("Cal/PID/TargetAngle", targetAngle);
            SmartDashboard.putBoolean("Cal/PID/StepPositive", false);
        }
        
        if (SmartDashboard.getBoolean("Cal/PID/StepNegative", false)) {
            targetAngle -= stepSize;
            SmartDashboard.putNumber("Cal/PID/TargetAngle", targetAngle);
            SmartDashboard.putBoolean("Cal/PID/StepNegative", false);
        }
        
        // ----- Check for PID changes -----
        double newP = calibration.getTurretP();
        double newI = calibration.getTurretI();
        double newD = calibration.getTurretD();
        
        if (newP != lastP || newI != lastI || newD != lastD) {
            // PID values changed - update motor configuration
            updateMotorPID(newP, newI, newD);
            lastP = newP;
            lastI = newI;
            lastD = newD;
            
            System.out.println("PID Updated: P=" + newP + " I=" + newI + " D=" + newD);
        }
        
        // ----- Apply target angle -----
        turret.setTargetAngle(targetAngle);
        
        // ----- Update status display -----
        double currentAngle = turret.getCurrentAngle();
        double error = targetAngle - currentAngle;
        
        SmartDashboard.putNumber("Cal/PID/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("Cal/PID/Error", error);
        SmartDashboard.putBoolean("Cal/PID/OnTarget", Math.abs(error) < 2.0);
        
        // Update calibration manager
        calibration.setCurrentTurretAngle(currentAngle);
        
        // Log for AdvantageScope graphing
        SmartDashboard.putNumber("Cal/Turret/TargetAngle", targetAngle);
        
        String status = String.format("Target: %.1f deg | Current: %.1f deg | Error: %.1f deg | P=%.3f I=%.3f D=%.3f",
            targetAngle, currentAngle, error, newP, newI, newD);
        SmartDashboard.putString("Cal/Status", status);
    }
    
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Cal/Status", "PID tuning ended");
        
        // Notify via Elastic with final values
        calibration.logSuccess("PID Tuning Complete", 
            String.format("Final: P=%.4f I=%.4f D=%.4f", lastP, lastI, lastD));
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until cancelled
    }
    
    // HELPER METHODS
    
    /**
     * Updates the motor's PID configuration live.
     * This allows seeing the effect immediately without restarting.
     */
    private void updateMotorPID(double p, double i, double d) {
        // Update the motor's PID gains in real-time
        turret.updatePIDGains(p, i, d);
        
        SmartDashboard.putString("Cal/PID/LastUpdate", 
            String.format("P=%.4f I=%.4f D=%.4f", p, i, d));
    }
}
