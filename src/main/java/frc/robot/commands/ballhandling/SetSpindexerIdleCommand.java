package frc.robot.commands.ballhandling;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpindexerSubsystem;

/**
 * Command to set the spindexer to idle mode (slow forward to stage balls).
 */
public class SetSpindexerIdleCommand extends Command {
    
    private final SpindexerSubsystem spindexer;
    
    /**
     * Creates a new SetSpindexerIdleCommand.
     * @param spindexer The spindexer subsystem to use
     */
    public SetSpindexerIdleCommand(SpindexerSubsystem spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }
    
    @Override
    public void initialize() {
        spindexer.setIdle();
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately, spindexer continues running
        return true;
    }
}
