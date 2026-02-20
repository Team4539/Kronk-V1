package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CalibrationManager;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretFeedSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Elastic;

/**
 * Comprehensive calibration command for tuning shooter AND turret together.
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
 * - Cal/FeedBall: Feed a ball into the shooter (button)
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
    private final TurretFeedSubsystem turretFeed; // Optional - can be null
    
    // CALIBRATION MANAGER
    
    private final CalibrationManager calibration;
    
    // CONSTRUCTOR
    
    /**
     * Creates a full shooter calibration command (without ball feeding).
     * 
     * @param turret The turret subsystem (for angle control)
     * @param shooter The shooter subsystem (for power control)
     * @param vision The vision subsystem (for target detection)
     */
    public FullShooterCalibrationCommand(
            TurretSubsystem turret, 
            ShooterSubsystem shooter, 
            VisionSubsystem vision) {
        this(turret, shooter, vision, null);
    }
    
    /**
     * Creates a full shooter calibration command with ball feeding support.
     * 
     * @param turret The turret subsystem (for angle control)
     * @param shooter The shooter subsystem (for power control)
     * @param vision The vision subsystem (for target detection)
     * @param turretFeed The turret feed subsystem for feeding balls (optional, can be null)
     */
    public FullShooterCalibrationCommand(
            TurretSubsystem turret, 
            ShooterSubsystem shooter, 
            VisionSubsystem vision,
            TurretFeedSubsystem turretFeed) {
        this.turret = turret;
        this.shooter = shooter;
        this.vision = vision;
        this.turretFeed = turretFeed;
        this.calibration = CalibrationManager.getInstance();
        
        // We require both turret and shooter for full control
        addRequirements(turret, shooter);
        if (turretFeed != null) {
            addRequirements(turretFeed);
        }
    }
    
    // COMMAND LIFECYCLE
    
    @Override
    public void initialize() {
        // Set calibration mode to manual
        SmartDashboard.putBoolean("Cal/Turret/UseManual", true);
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", true);
        
        // Feed ball button
        SmartDashboard.putBoolean("Cal/FeedBall", false);
        
        // Display instructions
        SmartDashboard.putString("Cal/Status", "FULL CALIBRATION MODE - Adjust sliders and shoot!");
        SmartDashboard.putString("Cal/Instructions", 
            "1. Use Cal/Turret/ManualAngle to aim. " +
            "2. Use Tuning/Shooter/TopRPM and BottomRPM to tune shot. " +
            "3. Click Cal/FeedBall to feed a ball. " +
            "4. Click Tuning/RecordPoint after a good shot.");
        
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
        
        // ----- Handle feed ball button -----
        if (turretFeed != null) {
            if (SmartDashboard.getBoolean("Cal/FeedBall", false)) {
                turretFeed.setShoot();
            } else {
                turretFeed.setIdle();
            }
        }
        
        // ----- Update status display -----
        updateStatusDisplay();
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop shooter and feed
        shooter.stop();
        if (turretFeed != null) {
            turretFeed.stop();
        }
        SmartDashboard.putBoolean("Cal/FeedBall", false);
        
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
        double turretAngle = SmartDashboard.getNumber("Cal/Turret/ManualAngle", 0.0);
        double topRPM = calibration.getShooterTopRPM();
        double bottomRPM = calibration.getShooterBottomRPM();
        
        String status;
        if (!vision.hasTarget()) {
            status = "WARNING: NO TARGET - Move to see AprilTags";
        } else if (distance <= 0) {
            status = String.format("WARNING: Tag visible, no pose");
        } else {
            status = String.format("OK Dist: %.2fm | Angle: %.1f deg | Top: %.0f RPM | Bot: %.0f RPM", 
                distance, turretAngle, topRPM, bottomRPM);
        }
        
        SmartDashboard.putString("Cal/Status", status);
        
        // Also show raw TX for reference
        if (vision.hasTarget()) {
            SmartDashboard.putNumber("Cal/RawTX", vision.getHorizontalOffset());
        }
    }
}
