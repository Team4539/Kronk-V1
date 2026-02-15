package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Interactive command for building the shooter power calibration table.
 * 
 * HOW TO USE:
 * 1. Position robot at known distance from hub
 * 2. Run this command
 * 3. Adjust "Calibration/TopPower" and "Calibration/BottomPower" in SmartDashboard
 * 4. Shoot until you find good settings
 * 5. Toggle "RecordShot" to log the calibration point
 * 6. Move to new distance and repeat
 * 7. Copy logged values to Constants.Shooter.SHOOTING_CALIBRATION
 */
public class ShootingCalibrationCommand extends Command {
    
    // Subsystems
    
    private final ShooterSubsystem shooter;
    private final LimelightSubsystem limelight;
    
    // State
    
    /** Current top motor power setting (0.0 - 1.0) */
    private double topPower = 0.5;
    
    /** Current bottom motor power setting (0.0 - 1.0) */
    private double bottomPower = 0.5;
    
    /** Flag indicating user wants to record current point */
    private boolean shouldRecord = false;
    
    /** Count of calibration points recorded */
    private int recordCount = 0;
    
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
        addRequirements(shooter);
    }

    // COMMAND LIFECYCLE

    @Override
    public void initialize() {
        // ----- Set up SmartDashboard controls -----
        SmartDashboard.putNumber("Calibration/TopPower", topPower);
        SmartDashboard.putNumber("Calibration/BottomPower", bottomPower);
        SmartDashboard.putBoolean("Calibration/RecordShot", false);
        SmartDashboard.putString("Calibration/Status", "Ready - Adjust powers and shoot");
        SmartDashboard.putNumber("Calibration/RecordCount", recordCount);
        
        // ----- Display instructions -----
        SmartDashboard.putString("Calibration/Instructions", 
            "1. Adjust TopPower (height) and BottomPower (distance). " +
            "2. Check 'RecordShot' after a good shot to log values.");
        
        System.out.println("=== SHOOTER CALIBRATION STARTED ===");
        System.out.println("Adjust powers via SmartDashboard");
    }

    @Override
    public void execute() {
        // ----- Read power settings from dashboard -----
        topPower = SmartDashboard.getNumber("Calibration/TopPower", topPower);
        bottomPower = SmartDashboard.getNumber("Calibration/BottomPower", bottomPower);
        
        // ----- Apply power to shooter motors -----
        shooter.setManualPower(topPower, bottomPower);
        
        // ----- Check for record request -----
        shouldRecord = SmartDashboard.getBoolean("Calibration/RecordShot", false);
        
        if (shouldRecord) {
            recordCalibrationPoint();
            SmartDashboard.putBoolean("Calibration/RecordShot", false);  // Reset toggle
        }
        
        // ----- Update status display -----
        double distance = limelight.hasPoseEstimate() ? limelight.getDistanceToHub() : -1;
        SmartDashboard.putNumber("Calibration/CurrentDistance", distance);
        SmartDashboard.putBoolean("Calibration/ShooterReady", shooter.isReady());
        
        if (distance < 0) {
            SmartDashboard.putString("Calibration/Status", "WARNING: No pose - distance unknown");
        } else {
            SmartDashboard.putString("Calibration/Status", 
                String.format("Distance: %.2fm | Top: %.2f | Bottom: %.2f", distance, topPower, bottomPower));
        }
    }
    
    // CALIBRATION RECORDING
    
    /**
     * Records the current distance and motor powers as a calibration point.
     * Outputs ready-to-paste Java code to the console and SmartDashboard.
     */
    private void recordCalibrationPoint() {
        // Validate that we have distance data
        double distance = limelight.hasPoseEstimate() ? limelight.getDistanceToHub() : -1;
        
        if (distance < 0) {
            SmartDashboard.putString("Calibration/LastRecord", "FAILED - No pose estimate!");
            System.out.println("!!! RECORD FAILED - No pose estimate available !!!");
            return;
        }
        
        recordCount++;
        
        // Format as ready-to-paste Java code
        String record = String.format("put(%.1f, new double[]{%.2f, %.2f});  // Record #%d", 
                                      distance, topPower, bottomPower, recordCount);
        
        // Update dashboard
        SmartDashboard.putString("Calibration/LastRecord", record);
        SmartDashboard.putNumber("Calibration/RecordCount", recordCount);
        SmartDashboard.putString("Calibration/Record" + recordCount, record);
        
        // Log to console for easy copy/paste
        System.out.println("+===============================================+");
        System.out.println("|       CALIBRATION POINT RECORDED (#" + recordCount + ")        |");
        System.out.println("+===============================================+");
        System.out.println("|  Distance:    " + String.format("%-10.2f", distance) + " meters          |");
        System.out.println("|  Top Power:   " + String.format("%-10.2f", topPower) + "                 |");
        System.out.println("|  Bottom Power:" + String.format("%-10.2f", bottomPower) + "                 |");
        System.out.println("+===============================================+");
        System.out.println("|  Add to Constants.java:                       |");
        System.out.println("|  " + record);
        System.out.println("+===============================================+");
    }

    // CLEANUP

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        SmartDashboard.putString("Calibration/Status", "Calibration ended - " + recordCount + " points recorded");
        
        System.out.println("=== SHOOTER CALIBRATION ENDED ===");
        System.out.println("Total points recorded: " + recordCount);
    }

    @Override
    public boolean isFinished() {
        // Runs until manually canceled
        return false;
    }
}
