package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Oscillates the intake pivot between retracted and idle positions
 * while the shooter is active. This "jiggles" balls to keep them
 * loose and feeding smoothly into the shooter.
 *
 * The pivot swings on a configurable cycle time (JIGGLE_CYCLE_SECONDS).
 * Rollers are left stopped — this is purely a pivot movement.
 *
 * Run this command .alongWith() AutoShootCommand while holding LT.
 * It ends when interrupted (button released).
 */
public class IntakeJiggleCommand extends Command {

    private final IntakeSubsystem intake;
    private double startTime;

    public IntakeJiggleCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        intake.stopRollers(); // No rolling, just jiggle the pivot
    }

    @Override
    public void execute() {
        double elapsed = Timer.getFPGATimestamp() - startTime;
        double cycleTime = Constants.Intake.JIGGLE_CYCLE_SECONDS;

        // Triangle wave: 0 → 1 → 0 over one cycle
        double phase = (elapsed % cycleTime) / cycleTime;
        double blend = (phase < 0.5) ? (phase * 2.0) : (2.0 - phase * 2.0);

        // Gentle oscillation: retracted position ± JIGGLE_AMPLITUDE_DEG
        double center = Constants.Intake.RETRACTED_ANGLE_DEG;
        double amplitude = Constants.Intake.JIGGLE_AMPLITUDE_DEG;
        double targetAngle = center + blend * amplitude;

        intake.setPivotAngle(targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        // Return to idle position when shooting stops
        intake.retract();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted (button released)
    }
}
