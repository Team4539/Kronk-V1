package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Tracks alliance color, game phase timing, scoring windows, and target selection.
 * 
 * Manages the alternating alliance shift system where each alliance has
 * designated 25-second scoring windows. Also handles shuttle mode
 * (automatic or manual) for switching between hub and trench targets.
 * 
 * Use Constants.SubsystemEnabled to disable subsystems for bench testing.
 */
public class GameStateManager {
    
    // --- Enums ---
    
    public enum TargetMode { HUB, TRENCH, DISABLED }
    
    public enum GamePhase {
        PRE_MATCH, AUTO, TRANSITION,
        SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4,
        END_GAME, POST_MATCH
    }
    
    // --- Singleton ---
    
    private static GameStateManager instance;
    
    public static GameStateManager getInstance() {
        if (instance == null) instance = new GameStateManager();
        return instance;
    }
    
    // --- State ---
    
    private Alliance robotAlliance = Alliance.Blue;
    private Alliance firstActiveAlliance = null;
    private boolean receivedGameMessage = false;
    private boolean justReceivedGameMessage = false;
    private TargetMode currentTargetMode = TargetMode.HUB;
    private GamePhase currentPhase = GamePhase.PRE_MATCH;
    private boolean forceShootEnabled = false;
    private boolean shuttleMode = false;
    private boolean shuttleModeManualOverride = false;
    private boolean inShuttleZone = false;
    
    private GameStateManager() {}
    
    // ========================================================================
    // UPDATE (call every cycle from RobotContainer)
    // ========================================================================
    
    /** Update with shuttle zone info for auto-switching. */
    public void update(boolean robotInShuttleZone) {
        justReceivedGameMessage = false; // Clear each cycle; set by updateFirstActiveAlliance()
        updateAllianceColor();
        updateFirstActiveAlliance();
        updateGamePhase();
        updateAutoShuttleMode(robotInShuttleZone);
        updateTargetMode();
        publishTelemetry();
    }
    
    /** Update without shuttle zone info. */
    public void update() {
        update(inShuttleZone);
    }
    
    // ========================================================================
    // PRIVATE UPDATE HELPERS
    // ========================================================================
    
    private void updateAllianceColor() {
        robotAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }
    
    private void updateFirstActiveAlliance() {
        if (receivedGameMessage) return;
        
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData != null && gameData.length() > 0) {
            char c = gameData.charAt(0);
            if (c == 'B') {
                firstActiveAlliance = Alliance.Blue;
                receivedGameMessage = true;
                justReceivedGameMessage = true;
            } else if (c == 'R') {
                firstActiveAlliance = Alliance.Red;
                receivedGameMessage = true;
                justReceivedGameMessage = true;
            }
        }
    }
    
    private void updateGamePhase() {
        if (DriverStation.isAutonomous()) {
            currentPhase = GamePhase.AUTO;
        } else if (DriverStation.isTeleop()) {
            double t = DriverStation.getMatchTime();
            // matchTime is -1 when no FMS is connected (practice / local testing).
            // Treat that as normal teleop with no shift timing — NOT endgame.
            if (t < 0) {
                currentPhase = GamePhase.SHIFT_1; // Generic active-teleop phase
            } else if (t > 130) {
                currentPhase = GamePhase.TRANSITION;
            } else if (t > 105) {
                currentPhase = GamePhase.SHIFT_1;
            } else if (t > 80) {
                currentPhase = GamePhase.SHIFT_2;
            } else if (t > 55) {
                currentPhase = GamePhase.SHIFT_3;
            } else if (t > 30) {
                currentPhase = GamePhase.SHIFT_4;
            } else if (t > 0) {
                currentPhase = GamePhase.END_GAME;
            } else {
                currentPhase = GamePhase.POST_MATCH;
            }
        } else {
            // Disabled
            if (currentPhase == GamePhase.END_GAME || currentPhase == GamePhase.POST_MATCH) {
                currentPhase = GamePhase.POST_MATCH;
            } else if (currentPhase != GamePhase.POST_MATCH) {
                currentPhase = GamePhase.PRE_MATCH;
            }
        }
    }
    
    private void updateAutoShuttleMode(boolean robotInShuttleZone) {
        this.inShuttleZone = robotInShuttleZone;
        
        if (!Constants.Field.AUTO_SHUTTLE_ENABLED || shuttleModeManualOverride) return;
        
        if (robotInShuttleZone && !shuttleMode) {
            shuttleMode = true;
        } else if (!robotInShuttleZone && shuttleMode) {
            shuttleMode = false;
        }
    }
    
    private void updateTargetMode() {
        boolean canScore = isOurAllianceActive();
        
        if (isGreenLightPreShift()) canScore = true;
        if (inShuttleZone) canScore = true;
        
        // No FMS = free practice, always allow shooting
        if (!DriverStation.isFMSAttached() && !Constants.Field.OVERRIDE_FMS_CHECK) {
            canScore = true;
        }
        
        if (!canScore && !forceShootEnabled) {
            currentTargetMode = TargetMode.DISABLED;
        } else if (shuttleMode) {
            currentTargetMode = TargetMode.TRENCH;
        } else {
            currentTargetMode = TargetMode.HUB;
        }
    }
    
    // ========================================================================
    // ALLIANCE TIMING
    // ========================================================================
    
    public Alliance getCurrentlyActiveAlliance() {
        if (!receivedGameMessage || firstActiveAlliance == null) return null;
        
        Alliance other = (firstActiveAlliance == Alliance.Blue) ? Alliance.Red : Alliance.Blue;
        
        switch (currentPhase) {
            case SHIFT_1: case SHIFT_3: return firstActiveAlliance;
            case SHIFT_2: case SHIFT_4: return other;
            default: return null; // Both active (auto, transition, endgame)
        }
    }
    
    public boolean isOurAllianceActive() {
        Alliance active = getCurrentlyActiveAlliance();
        return active == null || robotAlliance == active;
    }
    
    /** Seconds until our next active shift starts. 0 if already active. */
    public double getSecondsUntilOurNextShift() {
        if (!receivedGameMessage || firstActiveAlliance == null || isOurAllianceActive()) return 0;
        
        double t = DriverStation.getMatchTime();
        boolean weAreFirst = (robotAlliance == firstActiveAlliance);
        
        switch (currentPhase) {
            case TRANSITION: return weAreFirst ? Math.max(0, t - 130) : Math.max(0, t - 105);
            case SHIFT_1: return weAreFirst ? 0 : Math.max(0, t - 105);
            case SHIFT_2: return weAreFirst ? Math.max(0, t - 80) : 0;
            case SHIFT_3: return weAreFirst ? 0 : Math.max(0, t - 55);
            case SHIFT_4: return weAreFirst ? Math.max(0, t - 30) : 0;
            default: return 0;
        }
    }
    
    /** True when 5-3 seconds before our shift. Drivers should head to scoring position. */
    public boolean isHeadBackWarning() {
        if (isOurAllianceActive()) return false;
        double s = getSecondsUntilOurNextShift();
        return s > 3.0 && s <= 5.0;
    }
    
    /** True when 3-0 seconds before our shift. Pre-aim and pre-spool! */
    public boolean isGreenLightPreShift() {
        if (isOurAllianceActive()) return false;
        double s = getSecondsUntilOurNextShift();
        return s > 0.0 && s <= 3.0;
    }
    
    /** Can we shoot right now? */
    public boolean canShoot() {
        return isGreenLightPreShift() || currentTargetMode != TargetMode.DISABLED;
    }
    
    /** Whether auto-aim is enabled. Always returns true (no toggle exists). */
    public boolean isAutoAimEnabled() {
        return true;
    }
    
    /** Alias for getSecondsUntilOurNextShift() - seconds until our alliance is active. */
    public double getTimeUntilActive() {
        return getSecondsUntilOurNextShift();
    }
    
    /**
     * Returns how far through the current shift we are, as 0.0 → 1.0.
     * 0.0 = shift just started, 1.0 = shift is about to end.
     * Works for ANY shift (not alliance-specific) so LEDs can show urgency
     * regardless of which alliance is active.
     * Returns -1 if not in a shift phase (transition, endgame, no FMS, etc.).
     */
    public double getCurrentShiftProgress() {
        double t = DriverStation.getMatchTime();
        if (t < 0) return -1; // No FMS timing
        
        // Each shift is 25 seconds. Find how far through the current one we are.
        double shiftStart; // matchTime value at shift START (higher number = earlier in match)
        double shiftEnd;   // matchTime value at shift END
        switch (currentPhase) {
            case SHIFT_1: shiftStart = 130; shiftEnd = 105; break;
            case SHIFT_2: shiftStart = 105; shiftEnd = 80;  break;
            case SHIFT_3: shiftStart = 80;  shiftEnd = 55;  break;
            case SHIFT_4: shiftStart = 55;  shiftEnd = 30;  break;
            default: return -1; // Not in a shift
        }
        // matchTime counts DOWN, so progress = how far from start toward end
        double elapsed = shiftStart - t;   // 0 at start, 25 at end
        double duration = shiftStart - shiftEnd; // 25
        return Math.max(0.0, Math.min(1.0, elapsed / duration));
    }
    
    // ========================================================================
    // GETTERS
    // ========================================================================
    
    public Alliance getRobotAlliance() { return robotAlliance; }
    public GamePhase getGamePhase() { return currentPhase; }
    public TargetMode getTargetMode() { return currentTargetMode; }
    public boolean isShuttleMode() { return shuttleMode; }
    public boolean isForceShootEnabled() { return forceShootEnabled; }
    public boolean isShuttleModeManualOverride() { return shuttleModeManualOverride; }
    public boolean isMatchActive() { return currentPhase != GamePhase.PRE_MATCH && currentPhase != GamePhase.POST_MATCH; }
    public boolean hasReceivedGameMessage() { return receivedGameMessage; }
    public boolean didJustReceiveGameMessage() { return justReceivedGameMessage; }
    public Alliance getFirstActiveAlliance() { return firstActiveAlliance; }
    
    /** Returns true if our alliance goes first in the shift order. */
    public boolean doWeGoFirst() {
        return receivedGameMessage && firstActiveAlliance == robotAlliance;
    }
    
    /** Seconds remaining in the current active shift. 0 if not active. */
    public double getTimeRemainingActive() {
        if (!isOurAllianceActive()) return 0;
        double t = DriverStation.getMatchTime();
        // Each shift ends at the following matchTime boundaries:
        //   SHIFT_1 ends at 105, SHIFT_2 at 80, SHIFT_3 at 55, SHIFT_4 at 30
        // First alliance is active in SHIFT_1/3, second in SHIFT_2/4.
        // The guard above ensures we only reach our own shift's case.
        switch (currentPhase) {
            case SHIFT_1: return Math.max(0, t - 105); // First alliance active
            case SHIFT_2: return Math.max(0, t - 80);  // Second alliance active
            case SHIFT_3: return Math.max(0, t - 55);  // First alliance active
            case SHIFT_4: return Math.max(0, t - 30);  // Second alliance active
            case END_GAME: return Math.max(0, t);
            default: return 0;
        }
    }
    
    // ========================================================================
    // CONTROL
    // ========================================================================
    
    public void setShuttleMode(boolean enabled) {
        shuttleMode = enabled;
        shuttleModeManualOverride = true;
    }
    
    public void clearShuttleModeOverride() {
        shuttleModeManualOverride = false;
    }
    
    public void setForceShootEnabled(boolean enabled) {
        forceShootEnabled = enabled;
    }
    
    public void reset() {
        receivedGameMessage = false;
        justReceivedGameMessage = false;
        firstActiveAlliance = null;
        forceShootEnabled = false;
        shuttleMode = false;
        shuttleModeManualOverride = false;
        inShuttleZone = false;
        currentPhase = GamePhase.PRE_MATCH;
        currentTargetMode = TargetMode.DISABLED;
    }
    
    // ========================================================================
    // TELEMETRY (just the 7 things drivers actually need)
    // ========================================================================
    
    private void publishTelemetry() {
        SmartDashboard.putString("Match/Status", currentPhase.name());
        SmartDashboard.putBoolean("Match/ShootUnlocked", canShoot());
        SmartDashboard.putBoolean("Match/HeadBack", isHeadBackWarning());
        SmartDashboard.putBoolean("Match/GreenLight", isGreenLightPreShift());
        SmartDashboard.putString("Match/Target", currentTargetMode.name());
        SmartDashboard.putBoolean("Match/ForceShoot", forceShootEnabled);
        SmartDashboard.putBoolean("Match/ShuttleMode", shuttleMode);
        // Additional numeric telemetry for drive coach visibility
        SmartDashboard.putNumber("Match/Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Match/SecondsUntilOurShift", getSecondsUntilOurNextShift());
        SmartDashboard.putNumber("Match/TimeRemainingActive", getTimeRemainingActive());
    }
}
