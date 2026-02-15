package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.ActionState;

/**
 * Sets the LED action state for visual feedback.
 * This command runs indefinitely and should be used with other commands.
 */
public class SetLEDActionCommand extends Command {
    
    private final LEDSubsystem leds;
    private final ActionState action;
    
    /**
     * Creates a command to set the LED action state.
     * 
     * @param leds The LED subsystem
     * @param action The action state to display
     */
    public SetLEDActionCommand(LEDSubsystem leds, ActionState action) {
        this.leds = leds;
        this.action = action;
        // Note: We do NOT require the LED subsystem so this can run alongside other commands
    }
    
    @Override
    public void initialize() {
        if (leds != null) {
            leds.setAction(action);
        }
    }
    
    @Override
    public void execute() {
        // Keep setting the action in case it gets overridden
        if (leds != null) {
            leds.setAction(action);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Clear the action when this command ends
        if (leds != null) {
            leds.clearAction();
        }
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted or parent command ends
    }
}
