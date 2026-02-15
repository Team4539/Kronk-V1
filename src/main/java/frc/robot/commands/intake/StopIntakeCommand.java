package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to stop the intake rollers and optionally retract the intake.
 */
public class StopIntakeCommand extends Command {
    
    private final IntakeSubsystem intake;
    private final boolean autoRetract;
    
    /**
     * Creates a new StopIntakeCommand that only stops the rollers.
     * @param intake The intake subsystem to use
     */
    public StopIntakeCommand(IntakeSubsystem intake) {
        this(intake, false);
    }
    
    /**
     * Creates a new StopIntakeCommand.
     * @param intake The intake subsystem to use
     * @param autoRetract If true, also retract the intake automatically
     */
    public StopIntakeCommand(IntakeSubsystem intake, boolean autoRetract) {
        this.intake = intake;
        this.autoRetract = autoRetract;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        if (autoRetract) {
            intake.stopAndRetract();
        } else {
            intake.stopRollers();
        }
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately
        return true;
    }
}
