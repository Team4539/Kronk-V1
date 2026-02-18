package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.Elastic;

/**
 * LIMELIGHT CALIBRATION COMMAND
 * Calibration command for tuning Limelight camera mounting and vision trust.
 * 
 * This command allows live adjustment of:
 * - Camera mounting offsets (X, Y, Z position on robot)
 * - Camera pitch angle
 * - Vision measurement standard deviations (trust levels)
 * 
 * HOW TO USE:
 * 
 * CAMERA MOUNTING CALIBRATION:
 * 1. Place robot at a known position on the field
 * 2. Compare "Limelight/PoseX/Y" to your known position
 * 3. Adjust Cal/Limelight/XOffset, YOffset, ZOffset, Pitch
 * 4. Repeat until vision pose matches physical position
 * 
 * VISION TRUST CALIBRATION:
 * 1. Move the robot around while viewing the pose estimate
 * 2. If pose jumps around too much: INCREASE std dev values
 * 3. If pose is too slow to update: DECREASE std dev values
 * 4. Values of 0.1-0.5 are typical starting points
 * 
 * WHAT EACH VALUE DOES:
 * 
 * CAMERA_X_OFFSET: Forward/backward from robot center (+ = forward)
 * CAMERA_Y_OFFSET: Left/right from robot center (+ = left)
 * CAMERA_Z_OFFSET: Height from ground to camera lens
 * CAMERA_PITCH_DEGREES: Tilt angle (+ = tilted up)
 * 
 * VISION_STD_DEV_X: How much to trust X position (lower = trust more)
 * VISION_STD_DEV_Y: How much to trust Y position
 * VISION_STD_DEV_THETA: How much to trust rotation
 * 
 * ADVANTAGESCOPE TIPS:
 * 
 * 1. Open the "Limelight Field" widget to see pose visualization
 * 2. Compare Limelight pose to odometry pose
 * 3. If they disagree significantly, check camera mounting
 * 4. Graph std devs vs pose jitter to find optimal values
 * 
 * @author Team 4539
 */
public class LimelightCalibrationCommand extends Command {
    
    // SUBSYSTEMS
    
    private final LimelightSubsystem limelight;
    
    // STATE
    
    /** Known X position for validation (set via SmartDashboard) */
    private double knownX = 0.0;
    
    /** Known Y position for validation (set via SmartDashboard) */
    private double knownY = 0.0;
    
    // CONSTRUCTOR
    
    /**
     * Creates a Limelight calibration command.
     * 
     * @param limelight The limelight subsystem to calibrate
     */
    public LimelightCalibrationCommand(LimelightSubsystem limelight) {
        this.limelight = limelight;
        // Note: We don't require the limelight subsystem because we're just reading values
    }
    
    // COMMAND LIFECYCLE
    
    @Override
    public void initialize() {
        // Initialize known position sliders
        SmartDashboard.putNumber("Cal/Limelight/KnownX", 0.0);
        SmartDashboard.putNumber("Cal/Limelight/KnownY", 0.0);
        SmartDashboard.putBoolean("Cal/Limelight/ValidatePosition", false);
        
        SmartDashboard.putString("Cal/Status", "LIMELIGHT CALIBRATION - Adjust camera mounting and vision trust");
        SmartDashboard.putString("Cal/Instructions", 
            "1. Place robot at known field position. " +
            "2. Enter known X/Y in Cal/Limelight/KnownX/Y. " +
            "3. Adjust offsets until pose matches. " +
            "4. Tune std devs for smooth tracking.");
        
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Limelight Calibration",
            "Adjust camera offsets and vision trust values"));
    }
    
    @Override
    public void execute() {
        // ----- Read known position -----
        knownX = SmartDashboard.getNumber("Cal/Limelight/KnownX", knownX);
        knownY = SmartDashboard.getNumber("Cal/Limelight/KnownY", knownY);
        
        // ----- Get current vision pose -----
        boolean hasTarget = limelight.hasTarget();
        boolean hasPose = limelight.hasPoseEstimate();
        
        // ----- Update status display -----
        if (!hasTarget) {
            SmartDashboard.putString("Cal/Status", "WARNING: NO TARGET - Move to see AprilTags");
        } else if (!hasPose) {
            SmartDashboard.putString("Cal/Status", 
                String.format("WARNING: Tag %d visible, no pose estimate", limelight.getTargetId()));
        } else {
            double poseX = limelight.getRobotPose().getX();
            double poseY = limelight.getRobotPose().getY();
            double poseYaw = limelight.getRobotPose().getRotation().getDegrees();
            
            // Calculate error from known position
            double errorX = poseX - knownX;
            double errorY = poseY - knownY;
            double totalError = Math.sqrt(errorX * errorX + errorY * errorY);
            
            SmartDashboard.putNumber("Cal/Limelight/PoseX", poseX);
            SmartDashboard.putNumber("Cal/Limelight/PoseY", poseY);
            SmartDashboard.putNumber("Cal/Limelight/PoseYaw", poseYaw);
            SmartDashboard.putNumber("Cal/Limelight/ErrorX", errorX);
            SmartDashboard.putNumber("Cal/Limelight/ErrorY", errorY);
            SmartDashboard.putNumber("Cal/Limelight/TotalError", totalError);
            
            String status = String.format("OK Pose: (%.2f, %.2f) @ %.1f deg | Error from known: %.2fm",
                poseX, poseY, poseYaw, totalError);
            SmartDashboard.putString("Cal/Status", status);
        }
        
        // ----- Handle validation request -----
        if (SmartDashboard.getBoolean("Cal/Limelight/ValidatePosition", false)) {
            validatePosition();
            SmartDashboard.putBoolean("Cal/Limelight/ValidatePosition", false);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Cal/Status", "Limelight calibration ended");
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "Limelight Calibration Complete", ""));
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until cancelled
    }
    
    // HELPER METHODS
    
    private void validatePosition() {
        if (!limelight.hasPoseEstimate()) {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.ERROR, "Validation Failed", "No pose estimate available"));
            SmartDashboard.putString("Cal/Status", "VALIDATION FAILED: No pose estimate");
            return;
        }
        
        double poseX = limelight.getRobotPose().getX();
        double poseY = limelight.getRobotPose().getY();
        double errorX = poseX - knownX;
        double errorY = poseY - knownY;
        double totalError = Math.sqrt(errorX * errorX + errorY * errorY);
        
        String description = String.format("Known: (%.2f, %.2f) | Vision: (%.2f, %.2f) | Error: %.2fm", 
            knownX, knownY, poseX, poseY, totalError);
        
        Elastic.NotificationLevel level;
        String grade;
        if (totalError < 0.1) {
            level = Elastic.NotificationLevel.INFO;
            grade = "EXCELLENT (< 10cm)";
        } else if (totalError < 0.25) {
            level = Elastic.NotificationLevel.INFO;
            grade = "GOOD (< 25cm)";
        } else if (totalError < 0.5) {
            level = Elastic.NotificationLevel.WARNING;
            grade = "ACCEPTABLE - Consider fine-tuning";
        } else {
            level = Elastic.NotificationLevel.ERROR;
            grade = "POOR - Check camera mounting";
        }
        
        Elastic.sendNotification(new Elastic.Notification(level, grade, description));
        SmartDashboard.putString("Cal/Status", "VALIDATION: " + grade);
    }
}
