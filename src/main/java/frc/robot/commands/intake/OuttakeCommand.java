package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to spin the intake rollers in reverse to outtake game pieces.
 */
public class OuttakeCommand extends Command {
    
    private final IntakeSubsystem intake;
    
    /**
     * Creates a new OuttakeCommand.
     * @param intake The intake subsystem to use
     */
    public OuttakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.startOuttake();
    }
    
    @Override
    public void execute() {
        // Continue running outtake
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
