package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Deploys the intake and runs the rollers with automatic stall detection.
 * 
 * While held, this command:
 *  1. Deploys the pivot and runs rollers at intake speed.
 *  2. Monitors supply current and roller velocity every cycle.
 *  3. If the roller stalls (high current + low velocity for STALL_TIME_SECONDS),
 *     it reverses at full speed for REVERSE_TIME_SECONDS to clear the jam,
 *     then automatically resumes intaking.
 * 
 * Runs until interrupted (button released).
 */
public class DeployIntakeCommand extends Command {

    private enum State { INTAKING, REVERSING }

    private final IntakeSubsystem intake;

    private State state;
    private double stallStartTime;    // When stall conditions first became true
    private double reverseStartTime;  // When we began the reverse pulse

    public DeployIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        state = State.INTAKING;
        stallStartTime = 0;
        reverseStartTime = 0;
        intake.deploy();
        intake.startIntake();
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        switch (state) {
            case INTAKING:
                // Check for stall: high current AND low velocity
                double current = intake.getRollerSupplyCurrent();
                double velocity = Math.abs(intake.getRollerVelocityRPS());

                boolean isStalling = current > Constants.Intake.STALL_CURRENT_THRESHOLD_AMPS
                                  && velocity < Constants.Intake.STALL_VELOCITY_THRESHOLD_RPS;

                if (isStalling) {
                    if (stallStartTime == 0) {
                        // Start timing the stall
                        stallStartTime = now;
                    } else if (now - stallStartTime >= Constants.Intake.STALL_TIME_SECONDS) {
                        // Stall confirmed — reverse!
                        state = State.REVERSING;
                        reverseStartTime = now;
                        intake.setRollerSpeed(Constants.Intake.REVERSE_SPEED_RPM);
                        System.out.println("[Intake] STALL DETECTED — reversing ("
                                + String.format("%.1fA, %.1f rps", current, velocity) + ")");
                    }
                } else {
                    // Conditions cleared, reset stall timer
                    stallStartTime = 0;
                }
                break;

            case REVERSING:
                if (now - reverseStartTime >= Constants.Intake.REVERSE_TIME_SECONDS) {
                    // Done reversing — resume intake
                    state = State.INTAKING;
                    stallStartTime = 0;
                    intake.startIntake();
                    System.out.println("[Intake] Reverse complete — resuming intake");
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Clean up — stop rollers, retract (RetractIntakeCommand will also fire on button release,
        // but we stop rollers here in case this command ends for any other reason)
        intake.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted (button released)
    }
}
