package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to center the turret (point forward).
 * Sets the turret to 0 degrees (facing forward relative to robot).
 */
public class CenterTurretCommand extends Command {
    
    private final TurretSubsystem turret;
    private final boolean waitForTarget;
    
    /**
     * Creates a new CenterTurretCommand that returns immediately.
     * @param turret The turret subsystem to use
     */
    public CenterTurretCommand(TurretSubsystem turret) {
        this(turret, false);
    }
    
    /**
     * Creates a new CenterTurretCommand.
     * @param turret The turret subsystem to use
     * @param waitForTarget If true, command waits until turret is centered
     */
    public CenterTurretCommand(TurretSubsystem turret, boolean waitForTarget) {
        this.turret = turret;
        this.waitForTarget = waitForTarget;
        addRequirements(turret);
    }
    
    @Override
    public void initialize() {
        turret.setTargetAngle(0.0); // Center position
    }
    
    @Override
    public boolean isFinished() {
        if (waitForTarget) {
            return turret.isOnTarget();
        } else {
            return true;
        }
    }
}
