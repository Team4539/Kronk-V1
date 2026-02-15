package frc.robot.commands.ballhandling;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpindexerSubsystem;

/**
 * Command to stop the spindexer completely.
 */
public class StopSpindexerCommand extends Command {
    
    private final SpindexerSubsystem spindexer;
    
    /**
     * Creates a new StopSpindexerCommand.
     * @param spindexer The spindexer subsystem to use
     */
    public StopSpindexerCommand(SpindexerSubsystem spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }
    
    @Override
    public void initialize() {
        spindexer.stop();
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately
        return true;
    }
}
