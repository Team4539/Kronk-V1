package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to set the shooter to a specific distance and keep it running.
 * This is a default/continuous command that continuously adjusts shooter speed.
 */
public class SetShooterDistanceCommand extends Command {
    
    private final ShooterSubsystem shooter;
    private double distanceMeters;
    private final boolean useTrenchMode;
    
    /**
     * Creates a new SetShooterDistanceCommand for hub shooting.
     * @param shooter The shooter subsystem to use
     * @param distanceMeters Distance to target in meters
     */
    public SetShooterDistanceCommand(ShooterSubsystem shooter, double distanceMeters) {
        this(shooter, distanceMeters, false);
    }
    
    /**
     * Creates a new SetShooterDistanceCommand.
     * @param shooter The shooter subsystem to use
     * @param distanceMeters Distance to target in meters
     * @param useTrenchMode If true, use trench calibration instead of hub
     */
    public SetShooterDistanceCommand(ShooterSubsystem shooter, double distanceMeters, boolean useTrenchMode) {
        this.shooter = shooter;
        this.distanceMeters = distanceMeters;
        this.useTrenchMode = useTrenchMode;
        addRequirements(shooter);
    }
    
    /**
     * Update the target distance dynamically.
     * Useful for following a moving target.
     * @param distanceMeters New distance to target
     */
    public void setDistance(double distanceMeters) {
        this.distanceMeters = distanceMeters;
    }
    
    @Override
    public void execute() {
        if (useTrenchMode) {
            shooter.setForDistanceTrench(distanceMeters);
        } else {
            shooter.setForDistance(distanceMeters);
        }
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
