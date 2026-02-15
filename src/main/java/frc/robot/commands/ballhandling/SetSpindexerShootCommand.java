package frc.robot.commands.ballhandling;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpindexerSubsystem;

/**
 * Command to set the spindexer to shooting mode (fast forward to feed balls).
 */
public class SetSpindexerShootCommand extends Command {
    
    private final SpindexerSubsystem spindexer;
    
    /**
     * Creates a new SetSpindexerShootCommand.
     * @param spindexer The spindexer subsystem to use
     */
    public SetSpindexerShootCommand(SpindexerSubsystem spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }
    
    @Override
    public void initialize() {
        spindexer.setShoot();
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately, spindexer continues running
        return true;
    }
}
