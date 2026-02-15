package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to stop the turret at its current position.
 * Sets the target angle to the current angle to hold position.
 */
public class StopTurretCommand extends Command {
    
    private final TurretSubsystem turret;
    
    /**
     * Creates a new StopTurretCommand.
     * @param turret The turret subsystem to use
     */
    public StopTurretCommand(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }
    
    @Override
    public void initialize() {
        // Set target to current position to hold still
        turret.setTargetAngle(turret.getCurrentAngle());
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately, turret holds position
        return true;
    }
}
