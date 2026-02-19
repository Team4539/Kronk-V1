package frc.robot.commands.calibrations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Elastic;

/**
 * Calibration command for verifying vision camera pose estimation accuracy.
 * 
 * Allows you to compare the vision's reported robot position against a known
 * physical position to identify and correct mounting or configuration errors.
 * 
 * HOW TO USE:
 * 
 * POSITION VALIDATION:
 * 1. Place robot at a known position on the field
 * 2. Enter the known X/Y into Cal/vision/KnownX and KnownY
 * 3. Compare reported pose to your known position
 * 4. Click "Cal/vision/ValidatePosition" for an accuracy grade
 * 
 * WHAT EACH CAMERA OFFSET DOES (in Constants.vision):
 * 
 * CAMERA_X_OFFSET: Forward/backward from robot center (+ = forward)
 * CAMERA_Y_OFFSET: Left/right from robot center (+ = left)
 * CAMERA_Z_OFFSET: Height from ground to camera lens
 * CAMERA_PITCH_DEGREES: Tilt angle (+ = tilted up)
 * 
 * VISION TRUST (std dev values in Constants.vision):
 * 
 * Lower std dev = trust vision more, higher = trust odometry more.
 * Scaled automatically by distance (farther from tags = less trust).
 * Values of 0.1-0.5 are typical starting points.
 */
public class VisionCalibrationCommand extends Command {
    
    // SUBSYSTEMS
    
    private final VisionSubsystem vision;
    
    // STATE
    
    /** Known X position for validation (set via SmartDashboard) */
    private double knownX = 0.0;
    
    /** Known Y position for validation (set via SmartDashboard) */
    private double knownY = 0.0;
    
    // CONSTRUCTOR
    
    /**
     * Creates a vision calibration command.
     * 
     * @param vision The vision subsystem to calibrate
     */
    public VisionCalibrationCommand(VisionSubsystem vision) {
        this.vision = vision;
        // Note: We don't require the vision subsystem because we're just reading values
    }
    
    // COMMAND LIFECYCLE
    
    @Override
    public void initialize() {
        // Initialize known position sliders
        SmartDashboard.putNumber("Cal/vision/KnownX", 0.0);
        SmartDashboard.putNumber("Cal/vision/KnownY", 0.0);
        SmartDashboard.putBoolean("Cal/vision/ValidatePosition", false);
        
        SmartDashboard.putString("Cal/Status", "vision CALIBRATION - Compare vision pose to known position");
        SmartDashboard.putString("Cal/Instructions", 
            "1. Place robot at known field position. " +
            "2. Enter known X/Y in Cal/vision/KnownX/Y. " +
            "3. Click ValidatePosition for accuracy grade. " +
            "4. If error is high, check camera mounting in Constants.vision.");
        
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "vision Calibration",
            "Adjust camera offsets and vision trust values"));
    }
    
    @Override
    public void execute() {
        // ----- Read known position -----
        knownX = SmartDashboard.getNumber("Cal/vision/KnownX", knownX);
        knownY = SmartDashboard.getNumber("Cal/vision/KnownY", knownY);
        
        // ----- Get current vision pose -----
        boolean hasTarget = vision.hasTarget();
        boolean hasPose = vision.hasPoseEstimate();
        
        // ----- Update status display -----
        if (!hasTarget) {
            SmartDashboard.putString("Cal/Status", "WARNING: NO TARGET - Move to see AprilTags");
        } else if (!hasPose) {
            SmartDashboard.putString("Cal/Status", 
                String.format("WARNING: Tag %d visible, no pose estimate", vision.getTargetId()));
        } else {
            double poseX = vision.getRobotPose().getX();
            double poseY = vision.getRobotPose().getY();
            double poseYaw = vision.getRobotPose().getRotation().getDegrees();
            
            // Calculate error from known position
            double errorX = poseX - knownX;
            double errorY = poseY - knownY;
            double totalError = Math.sqrt(errorX * errorX + errorY * errorY);
            
            SmartDashboard.putNumber("Cal/vision/PoseX", poseX);
            SmartDashboard.putNumber("Cal/vision/PoseY", poseY);
            SmartDashboard.putNumber("Cal/vision/PoseYaw", poseYaw);
            SmartDashboard.putNumber("Cal/vision/ErrorX", errorX);
            SmartDashboard.putNumber("Cal/vision/ErrorY", errorY);
            SmartDashboard.putNumber("Cal/vision/TotalError", totalError);
            
            String status = String.format("OK Pose: (%.2f, %.2f) @ %.1f deg | Error from known: %.2fm",
                poseX, poseY, poseYaw, totalError);
            SmartDashboard.putString("Cal/Status", status);
        }
        
        // ----- Handle validation request -----
        if (SmartDashboard.getBoolean("Cal/vision/ValidatePosition", false)) {
            validatePosition();
            SmartDashboard.putBoolean("Cal/vision/ValidatePosition", false);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Cal/Status", "vision calibration ended");
        Elastic.sendNotification(new Elastic.Notification(
            Elastic.NotificationLevel.INFO, "vision Calibration Complete", ""));
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until cancelled
    }
    
    // HELPER METHODS
    
    private void validatePosition() {
        if (!vision.hasPoseEstimate()) {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.ERROR, "Validation Failed", "No pose estimate available"));
            SmartDashboard.putString("Cal/Status", "VALIDATION FAILED: No pose estimate");
            return;
        }
        
        double poseX = vision.getRobotPose().getX();
        double poseY = vision.getRobotPose().getY();
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
