package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Aggressively pumps the intake pivot back and forth while spinning rollers
 * inward to force-feed balls into the shooter during firing.
 *
 * The pivot sweeps from near-retracted (where balls enter the shooter) to
 * near-deployed (pushing balls back down) on a fast cycle. Rollers spin
 * inward the whole time to keep balls moving toward the shooter.
 *
 * Run this command .alongWith() AutoShootCommand while holding LB.
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
        // Start rollers spinning inward to force-feed balls
        intake.setRollerSpeed(Constants.Intake.JIGGLE_ROLLER_RPM);
    }

    @Override
    public void execute() {
        double elapsed = Timer.getFPGATimestamp() - startTime;
        double cycleTime = Constants.Intake.JIGGLE_CYCLE_SECONDS;

        // Triangle wave: 0 → 1 → 0 over one cycle
        double phase = (elapsed % cycleTime) / cycleTime;
        double blend = (phase < 0.5) ? (phase * 2.0) : (2.0 - phase * 2.0);

        // Sweep between min and max jiggle angles
        double minAngle = Constants.Intake.JIGGLE_MIN_ANGLE_DEG;
        double maxAngle = Constants.Intake.JIGGLE_MAX_ANGLE_DEG;
        double targetAngle = minAngle + blend * (maxAngle - minAngle);

        intake.setPivotAngle(targetAngle);
        
        // Keep rollers spinning inward
        intake.setRollerSpeed(Constants.Intake.JIGGLE_ROLLER_RPM);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopRollers();
        intake.retract();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted (button released)
    }
}
