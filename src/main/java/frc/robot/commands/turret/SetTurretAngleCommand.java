package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to set the turret to a specific angle.
 */
public class SetTurretAngleCommand extends Command {
    
    private final TurretSubsystem turret;
    private final double targetAngle;
    private final boolean waitForTarget;
    
    /**
     * Creates a new SetTurretAngleCommand that returns immediately.
     * @param turret The turret subsystem to use
     * @param targetAngle Target angle in degrees (-180 to +180)
     */
    public SetTurretAngleCommand(TurretSubsystem turret, double targetAngle) {
        this(turret, targetAngle, false);
    }
    
    /**
     * Creates a new SetTurretAngleCommand.
     * @param turret The turret subsystem to use
     * @param targetAngle Target angle in degrees (-180 to +180)
     * @param waitForTarget If true, command waits until turret reaches target
     */
    public SetTurretAngleCommand(TurretSubsystem turret, double targetAngle, boolean waitForTarget) {
        this.turret = turret;
        this.targetAngle = targetAngle;
        this.waitForTarget = waitForTarget;
        addRequirements(turret);
    }
    
    @Override
    public void initialize() {
        turret.setTargetAngle(targetAngle);
    }
    
    @Override
    public boolean isFinished() {
        if (waitForTarget) {
            return turret.isOnTarget();
        } else {
            // End immediately if not waiting
            return true;
        }
    }
}
