package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.ShootingCalculator;

/**
 * Continuously aims the turret at the current target (hub or trench).
 * 
 * Reads the latest shooting solution from ShootingCalculator, which is
 * updated every cycle by RobotContainer.updateVisionPose(). This command
 * just applies the calculated turret angle.
 * 
 * This is the DEFAULT turret command -- it runs continuously when no other
 * turret command (like AutoShootCommand) is active.
 */
public class AimTurretToPoseCommand extends Command {
    
    // SUBSYSTEMS
    
    private final TurretSubsystem turret;
    private final LimelightSubsystem limelight;
    private final ShootingCalculator shootingCalc;
    
    // STATE
    
    /** Target turret angle from ShootingCalculator (degrees, -180 to +180) */
    private double targetTurretAngle = 0.0;
    
    /** Distance from robot to current target */
    private double distanceToTarget = 0.0;
    
    /** Whether we're currently aiming at trench (true) or hub (false) */
    private boolean aimingAtTrench = false;

    // CONSTRUCTOR
    
    /**
     * Creates the pose-based turret aiming command.
     * 
     * @param turret The turret subsystem to control
     * @param limelight The limelight subsystem (used for pose availability checks)
     */
    public AimTurretToPoseCommand(TurretSubsystem turret, LimelightSubsystem limelight) {
        this.turret = turret;
        this.limelight = limelight;
        this.shootingCalc = ShootingCalculator.getInstance();
        
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
        String targetName = aimingAtTrench ? "TRENCH" : "HUB";
        
        // ----- Read shooting solution from ShootingCalculator -----
        // ShootingCalculator is updated every cycle from RobotContainer.updateVisionPose(),
        // so we just read the latest values here.
        targetTurretAngle = shootingCalc.getTurretAngle();
        distanceToTarget = shootingCalc.getDistance();
        
        // ----- Command turret to calculated angle -----
        turret.setTargetAngle(targetTurretAngle);
        
        // ----- Publish telemetry -----
        SmartDashboard.putString("AimPose/Status", "Tracking " + targetName);
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
