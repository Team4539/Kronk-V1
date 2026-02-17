package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to deploy the intake to the down position.
 */
public class DeployIntakeCommand extends Command {
    
    private final IntakeSubsystem intake;
    
    /**
     * Creates a new DeployIntakeCommand.
     * @param intake The intake subsystem to use
     */
    public DeployIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.deployAndIntake();
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately, intake continues moving in background
        return true;
    }
}
