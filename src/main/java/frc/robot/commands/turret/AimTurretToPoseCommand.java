package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.GameStateManager.TargetMode;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.PiShootingHelper;

/**
 * Aims the turret at the current target using the Raspberry Pi coprocessor.
 * 
 * All turret angle calculations are done on the Pi. This command publishes
 * the robot pose to the Pi via NetworkTables and reads back the turret angle.
 * If the Pi is unreachable, PiShootingHelper provides a fallback angle based
 * on pose geometry and the Constants calibration tables.
 * 
 * This is the DEFAULT turret command -- it runs continuously when no other
 * turret command (like AutoShootCommand) is active.
 */
public class AimTurretToPoseCommand extends Command {
    
    // SUBSYSTEMS
    
    private final TurretSubsystem turret;
    private final LimelightSubsystem limelight;
    private final PiShootingHelper piHelper;
    
    // STATE
    
    /** The turret angle received from Pi (or fallback) */
    private double targetTurretAngle = 0.0;
    
    /** Distance from robot to current target */
    private double distanceToTarget = 0.0;
    
    /** Whether we're currently aiming at trench (true) or hub (false) */
    private boolean aimingAtTrench = false;

    // CONSTRUCTOR
    
    /**
     * Creates the pose-based turret aiming command.
     * All processing is offloaded to the Raspberry Pi via PiShootingHelper.
     * 
     * @param turret The turret subsystem to control
     * @param limelight The limelight subsystem for pose estimation
     */
    public AimTurretToPoseCommand(TurretSubsystem turret, LimelightSubsystem limelight) {
        this.turret = turret;
        this.limelight = limelight;
        this.piHelper = PiShootingHelper.getInstance();
        
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

        // ----- Determine target mode -----
        aimingAtTrench = gameState.isShuttleMode();
        TargetMode targetMode = aimingAtTrench ? TargetMode.TRENCH : TargetMode.HUB;
        String targetName = aimingAtTrench ? "TRENCH" : "HUB";
        
        // ----- Send pose to Pi and get turret angle back -----
        Pose2d robotPose = limelight.getRobotPose();
        
        // Default command doesn't have drivetrain reference, so use zero speeds.
        // AutoShootCommand provides real chassis speeds for lead compensation.
        piHelper.update(robotPose, new ChassisSpeeds(), targetMode);
        
        // ----- Read Pi solution -----
        targetTurretAngle = piHelper.getTurretAngle();
        distanceToTarget = piHelper.getDistance();
        
        // ----- Command turret to calculated angle -----
        turret.setTargetAngle(targetTurretAngle);
        
        // ----- Publish telemetry -----
        String fallbackTag = piHelper.isUsingFallback() ? "[FALLBACK] " : "";
        SmartDashboard.putString("AimPose/Status", fallbackTag + "Tracking " + targetName);
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
