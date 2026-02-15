package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to retract the intake to the up position.
 */
public class RetractIntakeCommand extends Command {
    
    private final IntakeSubsystem intake;
    
    /**
     * Creates a new RetractIntakeCommand.
     * @param intake The intake subsystem to use
     */
    public RetractIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.retract();
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately, intake continues moving in background
        return true;
    }
}
