package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to start the intake rollers spinning to intake game pieces.
 * Can optionally deploy the intake automatically.
 */
public class IntakeCommand extends Command {
    
    private final IntakeSubsystem intake;
    private final boolean autoDeploy;
    
    /**
     * Creates a new IntakeCommand that only spins the rollers.
     * @param intake The intake subsystem to use
     */
    public IntakeCommand(IntakeSubsystem intake) {
        this(intake, false);
    }
    
    /**
     * Creates a new IntakeCommand.
     * @param intake The intake subsystem to use
     * @param autoDeploy If true, also deploy the intake automatically
     */
    public IntakeCommand(IntakeSubsystem intake, boolean autoDeploy) {
        this.intake = intake;
        this.autoDeploy = autoDeploy;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        if (autoDeploy) {
            intake.deployAndIntake();
        } else {
            intake.startIntake();
        }
    }
    
    @Override
    public void execute() {
        // Continue running intake
    }
    
    @Override
    public void end(boolean interrupted) {
        intake.stopRollers();
    }
    
    @Override
    public boolean isFinished() {
        // Run until interrupted
        return false;
    }
}
