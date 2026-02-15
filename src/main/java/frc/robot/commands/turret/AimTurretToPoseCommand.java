package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.ShootingTrainingManager;
import frc.robot.util.ShootingTrainingManager.TargetType;

/**
 * Aims the turret at the current target using robot pose estimation.
 * Automatically switches between hub and trench based on robot position.
 * Applies learned shooting training corrections for improved accuracy.
 */
public class AimTurretToPoseCommand extends Command {
    
    // SUBSYSTEMS
    
    private final TurretSubsystem turret;
    private final LimelightSubsystem limelight;
    private final ShootingTrainingManager trainingManager;
    
    // STATE
    
    /** The calculated turret angle to point at target */
    private double targetTurretAngle = 0.0;
    
    /** Distance from robot to current target (for shooter calculations) */
    private double distanceToTarget = 0.0;
    
    /** Whether we're currently aiming at trench (true) or hub (false) */
    private boolean aimingAtTrench = false;
    
    /** Training correction applied this frame */
    private double trainingCorrection = 0.0;
    
    /** Confidence in training data */
    private double trainingConfidence = 0.0;

    // CONSTRUCTOR
    
    /**
     * Creates the pose-based turret aiming command.
     * 
     * @param turret The turret subsystem to control
     * @param limelight The limelight subsystem for pose estimation
     */
    public AimTurretToPoseCommand(TurretSubsystem turret, LimelightSubsystem limelight) {
        this.turret = turret;
        this.limelight = limelight;
        this.trainingManager = ShootingTrainingManager.getInstance();
        
        // This command requires exclusive control of the turret
        addRequirements(turret);
    }

    // COMMAND LIFECYCLE
    
    @Override
    public void initialize() {
        SmartDashboard.putString("AimPose/Status", "Starting");
    }

    @Override
    public void execute() {
        // ----- Get GameStateManager for settings -----
        GameStateManager gameState = GameStateManager.getInstance();
        
        // ----- Check if auto-aim is disabled -----
        if (!gameState.isAutoAimEnabled()) {
            SmartDashboard.putString("AimPose/Status", "Auto-Aim Disabled");
            return;  // Don't move turret if auto-aim is disabled
        }
        
        // ----- Check for valid pose -----
        if (!limelight.hasPoseEstimate()) {
            SmartDashboard.putString("AimPose/Status", "No Pose - Holding");
            // Keep turret at last known position when we lose tracking
            return;
        }

        // ----- Get current target mode from GameStateManager -----
        aimingAtTrench = gameState.isShuttleMode();
        
        // ----- Get target position and calculate turret angle -----
        Translation2d targetPosition;
        String targetName;
        TargetType targetType;
        
        if (aimingAtTrench) {
            // Shuttle mode - aim at trench
            targetPosition = limelight.getTrenchPosition();
            targetTurretAngle = limelight.getTurretAngleToTrench();
            distanceToTarget = limelight.getDistanceToTrench();
            targetName = "TRENCH";
            targetType = TargetType.TRENCH;
        } else {
            // Normal mode - aim at hub
            targetPosition = limelight.getHubPosition();
            targetTurretAngle = limelight.getTurretAngleToHub();
            distanceToTarget = limelight.getDistanceToHub();
            targetName = "HUB";
            targetType = TargetType.HUB;
        }
        
        // ----- Apply calibration offset (tunable via SmartDashboard) -----
        double calibrationOffset = SmartDashboard.getNumber("AimPose/CalibrationOffset", 
                                                            Constants.Turret.GLOBAL_ANGLE_OFFSET_DEG);
        targetTurretAngle += calibrationOffset;
        
        // ----- Apply shooting training correction -----
        trainingCorrection = trainingManager.getPredictedAngleCorrection(targetType, distanceToTarget);
        trainingConfidence = trainingManager.getConfidence(targetType, distanceToTarget);
        targetTurretAngle += trainingCorrection;
        
        // ----- Command turret to calculated angle -----
        turret.setTargetAngle(targetTurretAngle);
        
        // ----- Publish telemetry -----
        SmartDashboard.putString("AimPose/Status", "Tracking " + targetName);
        SmartDashboard.putString("AimPose/TargetMode", targetName);
        SmartDashboard.putNumber("AimPose/TargetAngle", targetTurretAngle);
        SmartDashboard.putNumber("AimPose/CurrentAngle", turret.getCurrentAngle());
        SmartDashboard.putNumber("AimPose/Error", targetTurretAngle - turret.getCurrentAngle());
        SmartDashboard.putNumber("AimPose/Distance", distanceToTarget);
        SmartDashboard.putNumber("AimPose/TrainingCorrection", trainingCorrection);
        SmartDashboard.putNumber("AimPose/TrainingConfidence", trainingConfidence);
        SmartDashboard.putNumber("AimPose/RobotX", limelight.getRobotPose().getX());
        SmartDashboard.putNumber("AimPose/RobotY", limelight.getRobotPose().getY());
        SmartDashboard.putNumber("AimPose/RobotHeading", limelight.getRobotPose().getRotation().getDegrees());
        SmartDashboard.putNumber("AimPose/TargetX", targetPosition.getX());
        SmartDashboard.putNumber("AimPose/TargetY", targetPosition.getY());
        
        // Put calibration offset on dashboard (editable)
        SmartDashboard.putNumber("AimPose/CalibrationOffset", calibrationOffset);
    }
    
    // STATUS METHODS
    
    /**
     * Check if the turret is aimed at the target.
     * Useful for determining when it's safe to shoot.
     * 
     * @return true if turret angle error is within tolerance
     */
    public boolean isOnTarget() {
        double error = Math.abs(targetTurretAngle - turret.getCurrentAngle());
        return error < Constants.Turret.ON_TARGET_TOLERANCE_DEG;
    }
    
    /**
     * Get the current distance to the target (hub or trench).
     * Useful for shooter power calculations.
     * 
     * @return Distance in meters
     */
    public double getDistanceToTarget() {
        return distanceToTarget;
    }
    
    /**
     * Check if currently aiming at trench (shuttle mode).
     * @return true if aiming at trench, false if aiming at hub
     */
    public boolean isAimingAtTrench() {
        return aimingAtTrench;
    }

    // COMMAND LIFECYCLE (continued)
    
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("AimPose/Status", interrupted ? "Interrupted" : "Ended");
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until interrupted
        // It never finishes on its own
        return false;
    }
}
