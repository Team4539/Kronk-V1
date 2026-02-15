package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.ShootingTrainingManager;

/**
 * Aims turret at AprilTags using raw Limelight TX offset.
 * 
 * Simple reactive tracking that adjusts turret to center the visible tag.
 * Applies per-tag calibration offsets, global offset, and learned training corrections.
 * 
 * Use for reactive tracking. For predictive pose-based aiming, use AimTurretToPoseCommand.
 */
public class AimTurretToTagsCommand extends Command {
    
    private final TurretSubsystem turret;
    private final LimelightSubsystem limelight;
    private final GameStateManager gameState;
    private final ShootingTrainingManager trainingManager;

    public AimTurretToTagsCommand(TurretSubsystem turret, LimelightSubsystem limelight) {
        this.turret = turret;
        this.limelight = limelight;
        this.gameState = GameStateManager.getInstance();
        this.trainingManager = ShootingTrainingManager.getInstance();
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (!gameState.isAutoAimEnabled()) {
            SmartDashboard.putString("AimTags/Status", "DISABLED");
            return;
        }
        
        if (!limelight.hasTarget()) {
            SmartDashboard.putString("AimTags/Status", "No Target");
            return;
        }

        int tagId = limelight.getTargetId();
        double distance = limelight.getEstimatedDistanceToTag();
        double tx = limelight.getHorizontalOffset();
        
        // Build up the target angle with all corrections
        double tagOffset = turret.getAngleOffset(tagId);
        double globalOffset = Constants.Turret.GLOBAL_ANGLE_OFFSET_DEG;
        double trainingCorrection = trainingManager.getPredictedCorrection(tagId, distance);
        
        double currentAngle = turret.getCurrentAngle();
        double targetAngle = currentAngle + tx + tagOffset + globalOffset + trainingCorrection;
        
        turret.setTargetAngle(targetAngle);
        
        // Telemetry
        SmartDashboard.putString("AimTags/Status", "Tag " + tagId);
        SmartDashboard.putNumber("AimTags/TagID", tagId);
        SmartDashboard.putNumber("AimTags/TX", tx);
        SmartDashboard.putNumber("AimTags/Distance", distance);
        SmartDashboard.putNumber("AimTags/TrainingCorrection", trainingCorrection);
        SmartDashboard.putNumber("AimTags/TargetAngle", targetAngle);
        SmartDashboard.putNumber("AimTags/Error", targetAngle - currentAngle);
    }

    @Override
    public boolean isFinished() { return false; }
}
