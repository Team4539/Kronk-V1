package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.PiShootingHelper;

/**
 * Confirms a pending shot result for the Pi's training CSV.
 * 
 * Two-phase recording workflow:
 * 1. Robot automatically snapshots state when turret feed fires (Phase 1)
 * 2. This command confirms the result AFTER the operator watches the ball land (Phase 2)
 * 
 * The Pi pairs the frozen snapshot with the result and writes to CSV.
 * If the shot was a HIT, the Pi auto-retrains the model.
 * 
 * USAGE:
 * Bind to operator controller buttons:
 * - POV Up:    new RecordShotCommand("HIT")
 * - POV Right: new RecordShotCommand("MISS")
 * 
 * The shot is only recorded if there's a pending snapshot (fired in the last 15 seconds).
 */
public class RecordShotCommand extends Command {
    
    private final PiShootingHelper piHelper;
    private final String result;
    private int cycleCount;
    
    /**
     * @param hit Whether the shot was a hit (true) or miss (false)
     */
    public RecordShotCommand(boolean hit) {
        this(hit ? "HIT" : "MISS");
    }
    
    /**
     * @param result Custom result string ("HIT", "MISS", "LEFT", "RIGHT", "SHORT", "LONG")
     */
    public RecordShotCommand(String result) {
        this.piHelper = PiShootingHelper.getInstance();
        this.result = result;
    }
    
    @Override
    public void initialize() {
        cycleCount = 0;
        piHelper.confirmShot(result);
    }
    
    @Override
    public void execute() {
        cycleCount++;
    }
    
    @Override
    public boolean isFinished() {
        // Clear flag after 2 cycles (100ms at 50Hz) to ensure Pi sees the edge
        if (cycleCount >= 2) {
            piHelper.clearRecordFlag();
            return true;
        }
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        piHelper.clearRecordFlag();
    }
}
