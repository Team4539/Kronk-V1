package frc.robot.commands.ballhandling;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretFeedSubsystem;

/**
 * Command to set the turret feed to shooting mode (fast forward to dump balls).
 */
public class SetTurretFeedShootCommand extends Command {
    
    private final TurretFeedSubsystem turretFeed;
    
    /**
     * Creates a new SetTurretFeedShootCommand.
     * @param turretFeed The turret feed subsystem to use
     */
    public SetTurretFeedShootCommand(TurretFeedSubsystem turretFeed) {
        this.turretFeed = turretFeed;
        addRequirements(turretFeed);
    }
    
    @Override
    public void initialize() {
        turretFeed.setShoot();
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately, turret feed continues running
        return true;
    }
}
