package frc.robot.commands.ballhandling;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretFeedSubsystem;

/**
 * Command to stop the turret feed completely.
 */
public class StopTurretFeedCommand extends Command {
    
    private final TurretFeedSubsystem turretFeed;
    
    /**
     * Creates a new StopTurretFeedCommand.
     * @param turretFeed The turret feed subsystem to use
     */
    public StopTurretFeedCommand(TurretFeedSubsystem turretFeed) {
        this.turretFeed = turretFeed;
        addRequirements(turretFeed);
    }
    
    @Override
    public void initialize() {
        turretFeed.stop();
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately
        return true;
    }
}
