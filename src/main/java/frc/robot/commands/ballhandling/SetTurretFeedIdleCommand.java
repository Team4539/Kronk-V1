package frc.robot.commands.ballhandling;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretFeedSubsystem;

/**
 * Command to set the turret feed to idle mode (slow reverse to build up balls).
 */
public class SetTurretFeedIdleCommand extends Command {
    
    private final TurretFeedSubsystem turretFeed;
    
    /**
     * Creates a new SetTurretFeedIdleCommand.
     * @param turretFeed The turret feed subsystem to use
     */
    public SetTurretFeedIdleCommand(TurretFeedSubsystem turretFeed) {
        this.turretFeed = turretFeed;
        addRequirements(turretFeed);
    }
    
    @Override
    public void initialize() {
        turretFeed.setIdle();
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately, turret feed continues running
        return true;
    }
}
