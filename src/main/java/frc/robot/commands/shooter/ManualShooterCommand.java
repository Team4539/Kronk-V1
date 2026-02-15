package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to manually set shooter motor powers.
 * Useful for testing and calibration.
 */
public class ManualShooterCommand extends Command {
    
    private final ShooterSubsystem shooter;
    private final double topPower;
    private final double bottomPower;
    
    /**
     * Creates a new ManualShooterCommand.
     * @param shooter The shooter subsystem to use
     * @param topPower Power for top motor (0.0 to 1.0)
     * @param bottomPower Power for bottom motor (0.0 to 1.0)
     */
    public ManualShooterCommand(ShooterSubsystem shooter, double topPower, double bottomPower) {
        this.shooter = shooter;
        this.topPower = topPower;
        this.bottomPower = bottomPower;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        shooter.setManualPower(topPower, bottomPower);
    }
    
    @Override
    public void execute() {
        // Continuously set power to override any other changes
        shooter.setManualPower(topPower, bottomPower);
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
    
    @Override
    public boolean isFinished() {
        // Run continuously until interrupted
        return false;
    }
}
