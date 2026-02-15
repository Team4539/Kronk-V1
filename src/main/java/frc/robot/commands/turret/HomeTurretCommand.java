package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to home the turret to the zero position.
 * This resets the turret's encoder to establish a known reference point.
 * The robot operator should manually position the turret at the physical
 * zero/home position before running this command.
 */
public class HomeTurretCommand extends Command {
    
    private final TurretSubsystem turret;
    
    /**
     * Creates a new HomeTurretCommand.
     * @param turret The turret subsystem to use
     */
    public HomeTurretCommand(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }
    
    @Override
    public void initialize() {
        turret.resetPosition();
    }
    
    @Override
    public boolean isFinished() {
        // Command ends immediately after resetting
        return true;
    }
}
