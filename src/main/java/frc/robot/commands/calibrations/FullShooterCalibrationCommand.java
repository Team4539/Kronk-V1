package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.ShootingCalculator;

/**
 * Comprehensive ALL-IN-ONE calibration command for tuning shooter AND turret together.
 * 
 * Records unified calibration points that capture the full robot state:
 *   {robotX, robotY, bearingToTarget, turretAngleOffset, topRPM, bottomRPM}
 * 
 * This replaces the old separate shooting and rotation calibration workflows.
 * One button press records everything at once.
 * 
 * Provides full manual control over:
 * - Turret angle (via SmartDashboard slider)
 * - Top motor RPM (via SmartDashboard slider)
 * - Bottom motor RPM (via SmartDashboard slider)
 * 
 * Also displays:
 * - Current AprilTag detection (ID, TX offset)
 * - Distance to target
 * - Robot pose from vision
 * 
 * HOW TO USE:
 * 
 * 1. Position the robot where it can see AprilTags
 * 2. Run this command from SmartDashboard
 * 3. Use sliders to adjust:
 *    - Cal/Turret/ManualAngle: Point turret at target
 *    - Tuning/Shooter/TopRPM: Adjust shot arc/height
 *    - Tuning/Shooter/BottomRPM: Adjust shot distance
 * 4. Shoot a ball and observe
 * 5. When the shot is good, click "Cal/RecordShootingPoint"
 * 6. Move to a new distance and repeat
 * 7. When done, click "Cal/PrintShooterTable" to get Java code for Constants.java
 * 
 * DASHBOARD BUTTONS AVAILABLE DURING CALIBRATION:
 * 
 * - Cal/Turret/ManualAngle: Aim turret (slider)
 * - Tuning/Shooter/TopRPM: Shot arc/height (slider)
 * - Tuning/Shooter/BottomRPM: Shot distance (slider)
 * - POV Down on controller: Feed a ball into the shooter (hold button)
 * - Cal/RecordShootingPoint: Record current shot as calibration point (button)
 * - Cal/PrintShooterTable: Export all recorded points as Java code (button)
 * 
 * TIPS FOR CALIBRATION:
 * 
 * 1. Start at close range (~2m) and work outward
 * 2. Record 3-5 shots at each distance to verify consistency
 * 3. Small changes first! (50-100 RPM increments)
 * 4. TOP MOTOR = HEIGHT/ARC (more = higher shot)
 * 5. BOTTOM MOTOR = DISTANCE (more = farther shot)
 * 6. Turret angle offset can compensate for camera alignment issues
 */
public class FullShooterCalibrationCommand extends Command {
    
    // SUBSYSTEMS
    
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    
    // CALIBRATION MANAGER & SHOOTING CALCULATOR
    
    private final CalibrationManager calibration;
    private final ShootingCalculator shootingCalc;
    
    // CONSTRUCTOR
    
    /**
     * Creates a full shooter calibration command.
     * 
     * Feed is controlled externally via the operator's "Feed Shot" button (POV Down),
     * which avoids subsystem conflicts that would cancel calibration.
     * 
     * @param turret The turret subsystem (for angle control)
     * @param shooter The shooter subsystem (for power control)
     * @param vision The vision subsystem (for target detection)
     */
    public FullShooterCalibrationCommand(
            TurretSubsystem turret, 
            ShooterSubsystem shooter, 
            VisionSubsystem vision) {
        this.turret = turret;
        this.shooter = shooter;
        this.vision = vision;
        this.calibration = CalibrationManager.getInstance();
        this.shootingCalc = ShootingCalculator.getInstance();
        
        // We require turret and shooter for full control.
        // NOTE: We intentionally do NOT require turretFeed here so that the
        // operator's "Feed Shot" button (POV Down) can be used during calibration
        // without cancelling this command due to a subsystem conflict.
        addRequirements(turret, shooter);
    }
    
    // COMMAND LIFECYCLE
    
    @Override
    public void initialize() {
        // Set calibration mode to manual
        SmartDashboard.putBoolean("Cal/Turret/UseManual", true);
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", true);
        
        // Display instructions
        SmartDashboard.putString("Cal/Status", "UNIFIED CALIBRATION MODE - Adjust turret offset + RPMs, then record!");
        SmartDashboard.putString("Cal/Instructions", 
            "1. Use Cal/Turret/ManualAngle to aim turret. " +
            "2. Use Tuning/Turret/RotationOffset to fine-tune aim. " +
            "3. Use Tuning/Shooter/TopRPM and BottomRPM to tune shot. " +
            "4. Use POV Down on controller to feed a ball. " +
            "5. Click Tuning/RecordPoint to save (x, y, bearing, offset, RPMs).");
        
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Full Calibration Started",
            "Use sliders to tune turret + shooter."));
    }
    
    @Override
    public void execute() {
        // ----- Update distance from vision -----
        if (vision.hasPoseEstimate()) {
            calibration.setCurrentDistance(vision.getDistanceToHub());
        }
        
        // ----- Apply manual turret angle from slider -----
        double angle = SmartDashboard.getNumber("Cal/Turret/ManualAngle", 0.0);
        angle += calibration.getTurretAngleOffset();
        turret.setTargetAngle(angle);
        
        // ----- Apply manual shooter RPM -----
        double topRPM = calibration.getShooterTopRPM();
        double bottomRPM = calibration.getShooterBottomRPM();
        shooter.setTargetRPM(topRPM, bottomRPM);
        
        // NOTE: Feed is controlled by the operator's POV Down button, not by this command.
        // This avoids subsystem conflicts that would cancel calibration.
        
        // ----- Update status display -----
        updateStatusDisplay();
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop shooter
        shooter.stop();
        
        // Reset to auto mode
        SmartDashboard.putBoolean("Cal/Turret/UseManual", false);
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", false);
        
        SmartDashboard.putString("Cal/Status", "Calibration ended");
        
        int points = calibration.getRecordCount();
        if (points > 0) {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.INFO, "Calibration Complete",
                points + " points recorded. Click PrintShooterTable to export."));
        } else {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.WARNING, "Calibration Ended",
                "No points recorded."));
        }
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until cancelled
    }
    
    // HELPER METHODS
    
    private void updateStatusDisplay() {
        double distance = calibration.getCurrentDistance();
        double topRPM = calibration.getShooterTopRPM();
        double bottomRPM = calibration.getShooterBottomRPM();
        double rotationOffset = calibration.getTurretRotationOffset();
        double bearing = shootingCalc.getRawBearing();
        double relX = shootingCalc.getRelativeX();
        double relY = shootingCalc.getRelativeY();
        
        String status;
        if (!vision.hasTarget()) {
            status = "WARNING: NO TARGET - Move to see AprilTags";
        } else if (distance <= 0) {
            status = String.format("WARNING: Tag visible, no pose");
        } else {
            status = String.format("OK Dist: %.2fm | RelX: %.2f RelY: %.2f | Bearing: %.0f° | Offset: %.2f° | Top: %.0f Bot: %.0f RPM", 
                distance, relX, relY, bearing, rotationOffset, topRPM, bottomRPM);
        }
        
        SmartDashboard.putString("Cal/Status", status);
        SmartDashboard.putNumber("Cal/Robot/RelX", relX);
        SmartDashboard.putNumber("Cal/Robot/RelY", relY);
        SmartDashboard.putNumber("Cal/Robot/Bearing", bearing);
        SmartDashboard.putNumber("Cal/Robot/Distance", distance);
        
        // Also show raw TX for reference
        if (vision.hasTarget()) {
            SmartDashboard.putNumber("Cal/RawTX", vision.getHorizontalOffset());
        }
    }
}
