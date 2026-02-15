package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to spin up the shooter motors to a specific distance.
 * Waits until the shooter is spun up and ready before finishing.
 */
public class SpinUpShooterCommand extends Command {
    
    private final ShooterSubsystem shooter;
    private final double distanceMeters;
    private final boolean waitForReady;
    
    /**
     * Creates a new SpinUpShooterCommand that waits until ready.
     * @param shooter The shooter subsystem to use
     * @param distanceMeters Distance to target in meters
     */
    public SpinUpShooterCommand(ShooterSubsystem shooter, double distanceMeters) {
        this(shooter, distanceMeters, true);
    }
    
    /**
     * Creates a new SpinUpShooterCommand.
     * @param shooter The shooter subsystem to use
     * @param distanceMeters Distance to target in meters
     * @param waitForReady If true, command waits until shooter is ready
     */
    public SpinUpShooterCommand(ShooterSubsystem shooter, double distanceMeters, boolean waitForReady) {
        this.shooter = shooter;
        this.distanceMeters = distanceMeters;
        this.waitForReady = waitForReady;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        shooter.setForDistance(distanceMeters);
    }
    
    @Override
    public boolean isFinished() {
        if (waitForReady) {
            return shooter.isReady();
        } else {
            // End immediately if not waiting
            return true;
        }
    }
}
