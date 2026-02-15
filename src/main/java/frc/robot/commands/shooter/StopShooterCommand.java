package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to stop the shooter motors.
 */
public class StopShooterCommand extends Command {
    
    private final ShooterSubsystem shooter;
    
    /**
     * Creates a new StopShooterCommand.
     * @param shooter The shooter subsystem to use
     */
    public StopShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        shooter.stop();
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately
        return true;
    }
}
