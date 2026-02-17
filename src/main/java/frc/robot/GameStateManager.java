package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;

/**
 * Manages game state including alliance color, scoring windows, and target selection.
 * Singleton class - use GameStateManager.getInstance() to access.
 */
public class GameStateManager {
    
    // Enums
    
    /** What the robot should aim at */
    public enum TargetMode {
        HUB,        // Shooting at own alliance hub for points
        TRENCH,     // Shuttling balls to own alliance trench
        DISABLED    // Not shooting (alliance inactive or match not started)
    }
    
    /**
     * Current phase of the match.
     * Phases are based purely on match timer boundaries from Table 6-2.
     * Alliance-specific warnings (5-sec head back, 3-sec green light) are
     * handled by query methods, not by separate phases.
     */
    public enum GamePhase {
        PRE_MATCH,   // Before match starts
        AUTO,        // Autonomous period (0:20 -> 0:00)
        TRANSITION,  // Buffer between auto and teleop (2:20 -> 2:10, 10 sec)
        SHIFT_1,     // First active alliance (2:10 -> 1:45, 25 sec)
        SHIFT_2,     // Other alliance (1:45 -> 1:20, 25 sec)
        SHIFT_3,     // First active alliance again (1:20 -> 0:55, 25 sec)
        SHIFT_4,     // Other alliance again (0:55 -> 0:30, 25 sec)
        END_GAME,    // Both alliances active (0:30 -> 0:00, 30 sec)
        POST_MATCH   // After match ends
    }
    
    // SINGLETON INSTANCE
    
    private static GameStateManager instance;
    
    // STATE VARIABLES
    
    /** Our robot's alliance color */
    private Alliance robotAlliance = Alliance.Blue;
    
    /** The alliance that goes first in Shift 1 (from FMS game message) */
    private Alliance firstActiveAlliance = null;
    
    /** Whether we've received the game specific message from FMS */
    private boolean receivedGameMessage = false;
    
    /** Current target mode (hub, trench, or disabled) */
    private TargetMode currentTargetMode = TargetMode.HUB;
    
    /** Current match phase */
    private GamePhase currentPhase = GamePhase.PRE_MATCH;
    
    /** Override to allow shooting even when alliance is inactive */
    private boolean forceShootEnabled = false;
    
    /** When true, aim at trench instead of hub */
    private boolean shuttleMode = false;

    /** When true, shuttle mode is manually set and won't be auto-changed */
    private boolean shuttleModeManualOverride = false;

    /** Tracks if robot is currently in the shuttle zone (for auto-switching) */
    private boolean inShuttleZone = false;

    /** 
     * PRACTICE MATCH MODE: When enabled, uses alliance timing logic even without FMS.
     */
    private boolean practiceMatchMode = false;

    // Practice settings
    private boolean practiceShooterOnly = false;
    private boolean practiceTurretOnly = false;
    private boolean practiceDriveOnly = false;
    private boolean practiceVisionOnly = false;
    private boolean practiceDisableIntake = false;
    private boolean practiceSlowMotion = false;
    private double practiceSpeedLimit = 1.0;
    
    // Test mode settings
    private boolean testModeEnabled = false;
    private boolean testShooterEnabled = true;
    private boolean testTurretEnabled = true;
    private boolean testDriveEnabled = true;
    private boolean testIntakeEnabled = true;
    private boolean testVisionEnabled = true;
    private boolean testAutoAimEnabled = true;
    private boolean testAutoShuttleEnabled = true;
    
    // Test specific values
    private double testShooterRPM = 0;
    private double testTurretAngle = 0;
    private boolean testManualShooterControl = false;
    private boolean testManualTurretControl = false;
    
    // Constructor
    
    private GameStateManager() {
        initializePracticeTestDashboard();
    }

    /**
     * Initialize practice and test mode settings on SmartDashboard.
     */
    private void initializePracticeTestDashboard() {
        // Practice folder - Match simulation and drive practice
        DashboardHelper.putBoolean(Category.PRACTICE, "MatchMode", false);
        DashboardHelper.putString(Category.PRACTICE, "FirstAlliance", "Blue");
        DashboardHelper.putBoolean(Category.PRACTICE, "ForceAllianceActive", false);
        
        DashboardHelper.putBoolean(Category.PRACTICE, "ShooterOnly", false);
        DashboardHelper.putBoolean(Category.PRACTICE, "TurretOnly", false);
        DashboardHelper.putBoolean(Category.PRACTICE, "DriveOnly", false);
        DashboardHelper.putBoolean(Category.PRACTICE, "VisionOnly", false);
        
        DashboardHelper.putBoolean(Category.PRACTICE, "DisableShooter", false);
        DashboardHelper.putBoolean(Category.PRACTICE, "DisableTurret", false);
        DashboardHelper.putBoolean(Category.PRACTICE, "DisableDrive", false);
        DashboardHelper.putBoolean(Category.PRACTICE, "DisableIntake", false);
        
        // Speed/Safety Limits
        DashboardHelper.putBoolean(Category.PRACTICE, "SlowMotion", false);
        DashboardHelper.putNumber(Category.PRACTICE, "SpeedLimit", 1.0);
        DashboardHelper.putNumber(Category.PRACTICE, "TurretSpeedLimit", 1.0);
        DashboardHelper.putNumber(Category.PRACTICE, "ShooterSpeedLimit", 1.0);
        
        // Aiming Options
        DashboardHelper.putBoolean(Category.PRACTICE, "DisableAutoAim", false);
        DashboardHelper.putBoolean(Category.PRACTICE, "DisableAutoShuttle", false);
        DashboardHelper.putBoolean(Category.PRACTICE, "ManualTurretOnly", false);
        
        // Test folder - Component testing and debugging
        DashboardHelper.putBoolean(Category.TEST, "Enabled", false);
        
        DashboardHelper.putBoolean(Category.TEST, "ShooterEnabled", true);
        DashboardHelper.putBoolean(Category.TEST, "TurretEnabled", true);
        DashboardHelper.putBoolean(Category.TEST, "DriveEnabled", true);
        DashboardHelper.putBoolean(Category.TEST, "IntakeEnabled", true);
        DashboardHelper.putBoolean(Category.TEST, "VisionEnabled", true);
        
        DashboardHelper.putBoolean(Category.TEST, "AutoAimEnabled", true);
        DashboardHelper.putBoolean(Category.TEST, "AutoShuttleEnabled", true);
        DashboardHelper.putBoolean(Category.TEST, "PoseEstimationEnabled", true);
        DashboardHelper.putBoolean(Category.TEST, "VisionFusionEnabled", true);
        
        DashboardHelper.putBoolean(Category.TEST, "ManualShooterControl", false);
        DashboardHelper.putNumber(Category.TEST, "ManualShooterRPM", 0);
        DashboardHelper.putBoolean(Category.TEST, "ManualTurretControl", false);
        DashboardHelper.putNumber(Category.TEST, "ManualTurretAngle", 0);
        
        DashboardHelper.putNumber(Category.TEST, "ShooterTestRPM", 3000);
        DashboardHelper.putBoolean(Category.TEST, "RunShooterAtTestRPM", false);
        DashboardHelper.putBoolean(Category.TEST, "ShooterCoastMode", false);
        
        DashboardHelper.putNumber(Category.TEST, "TurretTestAngle", 0);
        DashboardHelper.putBoolean(Category.TEST, "MoveTurretToTestAngle", false);
        DashboardHelper.putBoolean(Category.TEST, "TurretCoastMode", false);
        DashboardHelper.putBoolean(Category.TEST, "TurretZeroSensors", false);
        
        DashboardHelper.putNumber(Category.TEST, "DriveTestSpeed", 0.5);
        DashboardHelper.putBoolean(Category.TEST, "DriveCoastMode", false);
        DashboardHelper.putBoolean(Category.TEST, "DriveZeroGyro", false);
        DashboardHelper.putBoolean(Category.TEST, "DriveZeroOdometry", false);
        
        DashboardHelper.putBoolean(Category.TEST, "VisionBypassChecks", false);
        DashboardHelper.putBoolean(Category.TEST, "VisionLogAllData", false);
        DashboardHelper.putBoolean(Category.TEST, "VisionForceTag", false);
        DashboardHelper.putNumber(Category.TEST, "VisionForcedTagID", 1);
        
        DashboardHelper.putBoolean(Category.TEST, "BypassSoftLimits", false);
        DashboardHelper.putBoolean(Category.TEST, "BypassCurrentLimits", false);
        DashboardHelper.putBoolean(Category.TEST, "BypassTemperatureLimits", false);
        
        // Presets
        DashboardHelper.putBoolean(Category.SETTINGS, "Presets/CompetitionMode", false);
        DashboardHelper.putBoolean(Category.SETTINGS, "Presets/PitTestMode", false);
        DashboardHelper.putBoolean(Category.SETTINGS, "Presets/DriveOnlyMode", false);
        DashboardHelper.putBoolean(Category.SETTINGS, "Presets/ShooterTuningMode", false);
        DashboardHelper.putBoolean(Category.SETTINGS, "Presets/TurretTuningMode", false);
        DashboardHelper.putBoolean(Category.SETTINGS, "Presets/VisionDebugMode", false);
        DashboardHelper.putBoolean(Category.SETTINGS, "Presets/SafeMode", false);
    }
    
    public static GameStateManager getInstance() {
        if (instance == null) {
            instance = new GameStateManager();
        }
        return instance;
    }
    
    // Main update method
    
    /**
     * Update all game state. Call this every periodic cycle.
     */
    public void update() {
        updatePracticeTestSettings();
        updatePracticeMatchMode();
        updateAllianceColor();
        updateFirstActiveAlliance();
        updateGamePhase();
        updateTargetMode();
        updatePresets();
        publishTelemetry();
    }

    /**
     * Update all game state with shuttle zone information.
     * Call this every periodic cycle for auto-shuttle functionality.
     * 
     * @param robotInShuttleZone true if robot is past the auto-shuttle boundary line
     */
    public void update(boolean robotInShuttleZone) {
        updatePracticeTestSettings();
        updatePracticeMatchMode();
        updateAllianceColor();
        updateFirstActiveAlliance();
        updateGamePhase();
        updateAutoShuttleMode(robotInShuttleZone);
        updateTargetMode();
        updatePresets();
        publishTelemetry();
    }

    /**
     * Read all practice and test settings from SmartDashboard.
     */
    private void updatePracticeTestSettings() {
        // --- Practice Settings ---
        practiceShooterOnly = DashboardHelper.getBoolean(Category.PRACTICE, "ShooterOnly", false);
        practiceTurretOnly = DashboardHelper.getBoolean(Category.PRACTICE, "TurretOnly", false);
        practiceDriveOnly = DashboardHelper.getBoolean(Category.PRACTICE, "DriveOnly", false);
        practiceVisionOnly = DashboardHelper.getBoolean(Category.PRACTICE, "VisionOnly", false);
        practiceDisableIntake = DashboardHelper.getBoolean(Category.PRACTICE, "DisableIntake", false);
        practiceSlowMotion = DashboardHelper.getBoolean(Category.PRACTICE, "SlowMotion", false);
        practiceSpeedLimit = DashboardHelper.getNumber(Category.PRACTICE, "SpeedLimit", 1.0);
        
        // --- Test Mode Settings ---
        testModeEnabled = DashboardHelper.getBoolean(Category.TEST, "Enabled", false);
        testShooterEnabled = DashboardHelper.getBoolean(Category.TEST, "ShooterEnabled", true);
        testTurretEnabled = DashboardHelper.getBoolean(Category.TEST, "TurretEnabled", true);
        testDriveEnabled = DashboardHelper.getBoolean(Category.TEST, "DriveEnabled", true);
        testIntakeEnabled = DashboardHelper.getBoolean(Category.TEST, "IntakeEnabled", true);
        testVisionEnabled = DashboardHelper.getBoolean(Category.TEST, "VisionEnabled", true);
        testAutoAimEnabled = DashboardHelper.getBoolean(Category.TEST, "AutoAimEnabled", true);
        testAutoShuttleEnabled = DashboardHelper.getBoolean(Category.TEST, "AutoShuttleEnabled", true);
        
        // --- Test Specific Values ---
        testShooterRPM = DashboardHelper.getNumber(Category.TEST, "ManualShooterRPM", 0);
        testTurretAngle = DashboardHelper.getNumber(Category.TEST, "ManualTurretAngle", 0);
        testManualShooterControl = DashboardHelper.getBoolean(Category.TEST, "ManualShooterControl", false);
        testManualTurretControl = DashboardHelper.getBoolean(Category.TEST, "ManualTurretControl", false);
    }

    /**
     * Handle preset mode selections.
     * When a preset is enabled, it configures multiple settings at once.
     */
    private void updatePresets() {
        // Competition Mode - Everything enabled, no testing features
        if (DashboardHelper.getBoolean(Category.SETTINGS, "Presets/CompetitionMode", false)) {
            DashboardHelper.putBoolean(Category.TEST, "Enabled", false);
            DashboardHelper.putBoolean(Category.PRACTICE, "SlowMotion", false);
            DashboardHelper.putNumber(Category.PRACTICE, "SpeedLimit", 1.0);
            DashboardHelper.putBoolean(Category.SETTINGS, "Presets/CompetitionMode", false); // Auto-reset
            System.out.println("=== PRESET: Competition Mode Activated ===");
        }
        
        // Pit Test Mode - Safe speeds, all mechanisms enabled
        if (DashboardHelper.getBoolean(Category.SETTINGS, "Presets/PitTestMode", false)) {
            DashboardHelper.putBoolean(Category.TEST, "Enabled", true);
            DashboardHelper.putBoolean(Category.PRACTICE, "SlowMotion", true);
            DashboardHelper.putNumber(Category.PRACTICE, "SpeedLimit", 0.25);
            DashboardHelper.putBoolean(Category.SETTINGS, "Presets/PitTestMode", false); // Auto-reset
            System.out.println("=== PRESET: Pit Test Mode Activated (25% speed) ===");
        }
        
        // Drive Only Mode
        if (DashboardHelper.getBoolean(Category.SETTINGS, "Presets/DriveOnlyMode", false)) {
            DashboardHelper.putBoolean(Category.PRACTICE, "DriveOnly", true);
            DashboardHelper.putBoolean(Category.PRACTICE, "DisableShooter", true);
            DashboardHelper.putBoolean(Category.PRACTICE, "DisableTurret", true);
            DashboardHelper.putBoolean(Category.PRACTICE, "DisableIntake", true);
            DashboardHelper.putBoolean(Category.SETTINGS, "Presets/DriveOnlyMode", false); // Auto-reset
            System.out.println("=== PRESET: Drive Only Mode Activated ===");
        }
        
        // Shooter Tuning Mode
        if (DashboardHelper.getBoolean(Category.SETTINGS, "Presets/ShooterTuningMode", false)) {
            DashboardHelper.putBoolean(Category.PRACTICE, "ShooterOnly", true);
            DashboardHelper.putBoolean(Category.TEST, "ManualShooterControl", true);
            DashboardHelper.putBoolean(Category.PRACTICE, "DisableDrive", true);
            DashboardHelper.putBoolean(Category.PRACTICE, "DisableTurret", false);
            DashboardHelper.putBoolean(Category.SETTINGS, "Presets/ShooterTuningMode", false); // Auto-reset
            System.out.println("=== PRESET: Shooter Tuning Mode Activated ===");
        }
        
        // Turret Tuning Mode
        if (DashboardHelper.getBoolean(Category.SETTINGS, "Presets/TurretTuningMode", false)) {
            DashboardHelper.putBoolean(Category.PRACTICE, "TurretOnly", true);
            DashboardHelper.putBoolean(Category.TEST, "ManualTurretControl", true);
            DashboardHelper.putBoolean(Category.PRACTICE, "DisableDrive", true);
            DashboardHelper.putBoolean(Category.PRACTICE, "DisableShooter", true);
            DashboardHelper.putBoolean(Category.SETTINGS, "Presets/TurretTuningMode", false); // Auto-reset
            System.out.println("=== PRESET: Turret Tuning Mode Activated ===");
        }
        
        // Vision Debug Mode
        if (DashboardHelper.getBoolean(Category.SETTINGS, "Presets/VisionDebugMode", false)) {
            DashboardHelper.putBoolean(Category.PRACTICE, "VisionOnly", true);
            DashboardHelper.putBoolean(Category.TEST, "VisionLogAllData", true);
            DashboardHelper.putBoolean(Category.SETTINGS, "Presets/VisionDebugMode", false); // Auto-reset
            System.out.println("=== PRESET: Vision Debug Mode Activated ===");
        }
        
        // Safe Mode - Everything at minimum power
        if (DashboardHelper.getBoolean(Category.SETTINGS, "Presets/SafeMode", false)) {
            DashboardHelper.putBoolean(Category.PRACTICE, "SlowMotion", true);
            DashboardHelper.putNumber(Category.PRACTICE, "SpeedLimit", 0.15);
            DashboardHelper.putNumber(Category.PRACTICE, "TurretSpeedLimit", 0.25);
            DashboardHelper.putNumber(Category.PRACTICE, "ShooterSpeedLimit", 0.25);
            DashboardHelper.putBoolean(Category.SETTINGS, "Presets/SafeMode", false); // Auto-reset
            System.out.println("=== PRESET: Safe Mode Activated (15% drive, 25% mechanisms) ===");
        }
    }

    /**
     * Read practice match mode setting from SmartDashboard.
     * Also reads which alliance goes first for practice matches.
     */
    private void updatePracticeMatchMode() {
        // Read practice match mode toggle (now in Practice folder)
        practiceMatchMode = DashboardHelper.getBoolean(Category.PRACTICE, "MatchMode", false);
        
        // For practice matches, read which alliance goes first from SmartDashboard
        // Default to Blue if not set
        if (practiceMatchMode && !receivedGameMessage) {
            String practiceFirstAlliance = DashboardHelper.getString(Category.PRACTICE, "FirstAlliance", "Blue");
            if (practiceFirstAlliance.equalsIgnoreCase("Red")) {
                firstActiveAlliance = Alliance.Red;
            } else {
                firstActiveAlliance = Alliance.Blue;
            }
            receivedGameMessage = true;
            System.out.println("=== PRACTICE MATCH: " + firstActiveAlliance + " alliance active first ===");
        }
    }

    /**
     * Updates shuttle mode automatically based on robot position.
     * Only changes mode if auto-shuttle is enabled and manual override is not active.
     */
    private void updateAutoShuttleMode(boolean robotInShuttleZone) {
        this.inShuttleZone = robotInShuttleZone;
        
        // Check if auto-shuttle is enabled via SmartDashboard
        boolean autoShuttleEnabled = DashboardHelper.getBoolean(Category.SETTINGS, "Aim/AutoShuttleEnabled", Constants.Field.AUTO_SHUTTLE_ENABLED);
        
        // Only auto-switch if enabled and no manual override
        if (autoShuttleEnabled && !shuttleModeManualOverride) {
            // Auto-switch shuttle mode based on zone
            if (robotInShuttleZone && !shuttleMode) {
                shuttleMode = true;
                System.out.println("=== AUTO-SHUTTLE: Entering shuttle zone, switching to TRENCH ===");
            } else if (!robotInShuttleZone && shuttleMode) {
                shuttleMode = false;
                System.out.println("=== AUTO-SHUTTLE: Leaving shuttle zone, switching to HUB ===");
            }
        }
    }
    
    // PRIVATE UPDATE HELPERS
    
    /**
     * Read our alliance color from Driver Station.
     */
    private void updateAllianceColor() {
        robotAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }
    
    /**
     * Read the game specific message to determine who is active first.
     * Only reads once - the message tells us the initial state.
     */
    private void updateFirstActiveAlliance() {
        // Only read once per match
        if (receivedGameMessage) {
            return;
        }
        
        String gameData = DriverStation.getGameSpecificMessage();
        
        if (gameData != null && gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    // Blue alliance is active FIRST in Shift 1
                    firstActiveAlliance = Alliance.Blue;
                    receivedGameMessage = true;
                    System.out.println("=== GAME MESSAGE: Blue active first in Shift 1 ===");
                    break;
                case 'R':
                    // Red alliance is active FIRST in Shift 1
                    firstActiveAlliance = Alliance.Red;
                    receivedGameMessage = true;
                    System.out.println("=== GAME MESSAGE: Red active first in Shift 1 ===");
                    break;
                default:
                    // Invalid data - wait for valid message
                    System.out.println("=== GAME MESSAGE: Corrupt data: " + gameData + " ===");
                    break;
            }
        }
    }
    
    /**
     * Determine current match phase based on game mode and match time.
     * Timer boundaries from Table 6-2: MATCH SHIFTS.
     * Teleop match timer counts DOWN from 140 seconds (2:20).
     */
    private void updateGamePhase() {
        // Handle autonomous
        if (DriverStation.isAutonomous()) {
            currentPhase = GamePhase.AUTO;
            return;
        }
        
        // Handle teleop
        if (DriverStation.isTeleop()) {
            // Teleop - match time counts DOWN from 140 seconds (2:20)
            double matchTime = DriverStation.getMatchTime();
            
            // Determine phase based on time remaining (Table 6-2 boundaries)
            if (matchTime > 130) {           // 2:20 -> 2:10 (TRANSITION SHIFT, 10 sec)
                currentPhase = GamePhase.TRANSITION;
            } else if (matchTime > 105) {    // 2:10 -> 1:45 (SHIFT 1, 25 sec)
                currentPhase = GamePhase.SHIFT_1;
            } else if (matchTime > 80) {     // 1:45 -> 1:20 (SHIFT 2, 25 sec)
                currentPhase = GamePhase.SHIFT_2;
            } else if (matchTime > 55) {     // 1:20 -> 0:55 (SHIFT 3, 25 sec)
                currentPhase = GamePhase.SHIFT_3;
            } else if (matchTime > 30) {     // 0:55 -> 0:30 (SHIFT 4, 25 sec)
                currentPhase = GamePhase.SHIFT_4;
            } else if (matchTime > 0) {      // 0:30 -> 0:00 (END GAME, 30 sec)
                currentPhase = GamePhase.END_GAME;
            } else {
                // Match time hit zero - match is over
                currentPhase = GamePhase.POST_MATCH;
            }
            return;
        }
        
        // Handle disabled (pre-match or post-match)
        if (DriverStation.isDisabled()) {
            // Check if we're attached to FMS or in a test match
            if (DriverStation.isFMSAttached()) {
                // FMS attached - check match time to determine pre vs post
                double matchTime = DriverStation.getMatchTime();
                if (matchTime < 0 || matchTime == 0) {
                    // Match time is zero or negative - post match
                    currentPhase = GamePhase.POST_MATCH;
                } else {
                    // Match time is positive - pre match
                    currentPhase = GamePhase.PRE_MATCH;
                }
            } else {
                // Not attached to FMS - check if we ever received game message
                // If we received it and went through a match, we're post-match
                if (receivedGameMessage && currentPhase == GamePhase.POST_MATCH) {
                    // Stay in post-match if we were already there
                    currentPhase = GamePhase.POST_MATCH;
                } else if (currentPhase == GamePhase.END_GAME || 
                          currentPhase == GamePhase.SHIFT_4 || 
                          currentPhase == GamePhase.POST_MATCH) {
                    // Just finished a match - transition to post
                    currentPhase = GamePhase.POST_MATCH;
                } else {
                    // Default to pre-match
                    currentPhase = GamePhase.PRE_MATCH;
                }
            }
            return;
        }
        
        // Fallback - if not auto, teleop, or disabled, assume pre-match
        currentPhase = GamePhase.PRE_MATCH;
    }
    
    /**
     * Determine which alliance is currently active based on phase and game message.
     * @return The active alliance, or null if BOTH alliances are active
     */
    private Alliance getCurrentlyActiveAlliance() {
        // If we haven't received the game message, assume both active (safe default)
        if (!receivedGameMessage || firstActiveAlliance == null) {
            return null;  // null = both active
        }
        
        // Determine the "other" alliance (opposite of first active)
        Alliance otherAlliance = (firstActiveAlliance == Alliance.Blue) ? Alliance.Red : Alliance.Blue;
        
        switch (currentPhase) {
            case AUTO:
            case TRANSITION:
            case END_GAME:
                return null;  // Both alliances active during these phases
                
            case SHIFT_1:
            case SHIFT_3:
                return firstActiveAlliance;  // First alliance active (odd shifts)
                
            case SHIFT_2:
            case SHIFT_4:
                return otherAlliance;  // Other alliance active (even shifts)
                
            default:
                return null;  // Pre/post match - treat as both active
        }
    }
    
    /**
     * Update target mode based on alliance activity and user settings.
     * Also allows shooting during the 3-second green light pre-shift window.
     */
    @SuppressWarnings("unused")
    private void updateTargetMode() {
        // Check if our alliance is currently allowed to score
        boolean canScore = isOurAllianceActive();
        
        // GREEN LIGHT PRE-SHIFT: Allow shooting 3 seconds before our shift starts
        if (isGreenLightPreShift()) {
            canScore = true;
        }
        
        // SHUTTLE ZONE: If in shuttle zone, always allow shooting (shuttling to trench)
        if (inShuttleZone) {
            canScore = true;
        }
        
        // PRACTICE MATCH MODE: Use alliance timing logic (canScore already set correctly above)
        // Only bypass if NOT in practice match mode and no FMS
        if (!practiceMatchMode && !DriverStation.isFMSAttached() && !Constants.Field.OVERRIDE_FMS_CHECK) {
            // Free practice mode - always allow shooting
            canScore = true;
        }
        // If practiceMatchMode is true, we use the alliance timing logic even without FMS
        
        // Determine target mode
        // NOTE: shuttleMode is managed exclusively by updateAutoShuttleMode() and setShuttleMode().
        // This method only READS shuttleMode to determine the target, never writes it.
        if (!canScore && !forceShootEnabled) {
            currentTargetMode = TargetMode.DISABLED;
        } else if (shuttleMode) {
            currentTargetMode = TargetMode.TRENCH;
        } else {
            currentTargetMode = TargetMode.HUB;
        }
    }
    
    // PUBLIC QUERY METHODS
    
    /**
     * Check if our alliance is currently active (allowed to score).
     * @return true if our alliance can score right now
     */
    public boolean isOurAllianceActive() {
        Alliance activeAlliance = getCurrentlyActiveAlliance();
        
        // If null, both alliances are active
        if (activeAlliance == null) {
            return true;
        }
        
        // Check if our alliance matches the active alliance
        return robotAlliance == activeAlliance;
    }
    
    /**
     * Check if the opponent alliance is currently active.
     * @return true if opponent alliance is active
     */
    public boolean isOpponentAllianceActive() {
        Alliance activeAlliance = getCurrentlyActiveAlliance();
        
        if (activeAlliance == null) {
            return true;  // Both active
        }
        
        return robotAlliance != activeAlliance;
    }
    
    /**
     * Get our robot's alliance color.
     * @return Our Alliance (Blue or Red)
     */
    public Alliance getRobotAlliance() {
        return robotAlliance;
    }
    
    /**
     * Get which alliance is active first in Shift 1.
     * @return The first-active Alliance, or null if not yet received
     */
    public Alliance getFirstActiveAlliance() {
        return firstActiveAlliance;
    }
    
    /**
     * Get the currently active alliance.
     * @return The active Alliance, or null if BOTH are active
     */
    public Alliance getActiveAlliance() {
        return getCurrentlyActiveAlliance();
    }
    
    /**
     * Check if we've received the game specific message from FMS.
     * @return true if message has been received
     */
    public boolean hasReceivedGameMessage() {
        return receivedGameMessage;
    }
    
    /**
     * Get the current game phase.
     * @return Current GamePhase
     */
    public GamePhase getGamePhase() {
        return currentPhase;
    }
    
    /**
     * Get the current target mode.
     * @return Current TargetMode (HUB, TRENCH, or DISABLED)
     */
    public TargetMode getTargetMode() {
        return currentTargetMode;
    }
    
    /**
     * Check if match is in pre-match phase (before match starts).
     * @return true if in PRE_MATCH phase
     */
    public boolean isPreMatch() {
        return currentPhase == GamePhase.PRE_MATCH;
    }
    
    /**
     * Check if match is in post-match phase (after match ends).
     * @return true if in POST_MATCH phase
     */
    public boolean isPostMatch() {
        return currentPhase == GamePhase.POST_MATCH;
    }
    
    /**
     * Check if match is currently active (auto, teleop, or endgame).
     * @return true if match is actively running
     */
    public boolean isMatchActive() {
        return currentPhase != GamePhase.PRE_MATCH && currentPhase != GamePhase.POST_MATCH;
    }
    
    // PUBLIC CONTROL METHODS
    
    /**
     * Enable/disable shuttle mode (aim at trench instead of hub).
     * When called manually, this sets a manual override that prevents auto-switching.
     * Use clearShuttleModeOverride() to re-enable auto-switching.
     * @param enabled true to aim at trench, false for hub
     */
    public void setShuttleMode(boolean enabled) {
        this.shuttleMode = enabled;
        this.shuttleModeManualOverride = true;  // Manual toggle disables auto-switch
    }

    /**
     * Clear the manual override for shuttle mode.
     * After calling this, auto-shuttle will resume based on robot position.
     */
    public void clearShuttleModeOverride() {
        this.shuttleModeManualOverride = false;
        System.out.println("=== Shuttle mode override cleared, auto-shuttle resumed ===");
    }

    /**
     * Check if manual shuttle mode override is active.
     * @return true if auto-shuttle is disabled due to manual toggle
     */
    public boolean isShuttleModeManualOverride() {
        return shuttleModeManualOverride;
    }
    
    /**
     * Check if shuttle mode is enabled.
     * @return true if aiming at trench
     */
    public boolean isShuttleMode() {
        return shuttleMode;
    }

    /**
     * Check if robot is currently in the shuttle zone.
     * @return true if past the auto-shuttle boundary line
     */
    public boolean isInShuttleZone() {
        return inShuttleZone;
    }
    
    /**
     * Enable/disable force shoot (override alliance inactive restriction).
     * USE WITH CAUTION - may cause penalties if alliance is actually inactive!
     * @param enabled true to allow shooting regardless of alliance status
     */
    public void setForceShootEnabled(boolean enabled) {
        this.forceShootEnabled = enabled;
    }
    
    /**
     * Check if force shoot is enabled.
     */
    public boolean isForceShootEnabled() {
        return forceShootEnabled;
    }

    // Practice & Test Mode Getters
    
    public boolean isPracticeMatchMode() {
        return practiceMatchMode;
    }
    
    /**
     * Check if we're in "shooter only" practice mode.
     */
    public boolean isPracticeShooterOnly() {
        return practiceShooterOnly;
    }
    
    /**
     * Check if we're in "turret only" practice mode.
     */
    public boolean isPracticeTurretOnly() {
        return practiceTurretOnly;
    }
    
    /**
     * Check if we're in "drive only" practice mode.
     */
    public boolean isPracticeDriveOnly() {
        return practiceDriveOnly;
    }
    
    /**
     * Check if we're in "vision only" practice mode.
     */
    public boolean isPracticeVisionOnly() {
        return practiceVisionOnly;
    }
    
    /**
     * Check if slow motion practice mode is enabled.
     */
    public boolean isPracticeSlowMotion() {
        return practiceSlowMotion;
    }
    
    /**
     * Get the current practice speed limit (0.0 to 1.0).
     */
    public double getPracticeSpeedLimit() {
        return practiceSpeedLimit;
    }
    
    // --- Test Mode Checks ---
    
    /**
     * Check if test mode is enabled.
     */
    public boolean isTestModeEnabled() {
        return testModeEnabled;
    }
    
    /**
     * Check if shooter is enabled in test mode.
     * Returns true if test mode is disabled OR shooter is enabled in test mode.
     */
    public boolean isShooterEnabled() {
        if (!testModeEnabled) return true;
        return testShooterEnabled;
    }
    
    /**
     * Check if turret is enabled in test mode.
     * Returns true if test mode is disabled OR turret is enabled in test mode.
     */
    public boolean isTurretEnabled() {
        if (!testModeEnabled) return true;
        return testTurretEnabled;
    }
    
    /**
     * Check if drive is enabled in test mode.
     * Returns true if test mode is disabled OR drive is enabled in test mode.
     */
    public boolean isDriveEnabled() {
        if (!testModeEnabled) return true;
        return testDriveEnabled;
    }
    
    /**
     * Check if intake is enabled.
     * Considers both practice disable and test mode settings.
     */
    public boolean isIntakeEnabled() {
        if (practiceDisableIntake) return false;
        if (!testModeEnabled) return true;
        return testIntakeEnabled;
    }
    
    /**
     * Check if vision is enabled in test mode.
     */
    public boolean isVisionEnabled() {
        if (!testModeEnabled) return true;
        return testVisionEnabled;
    }
    
    /**
     * Check if auto-aim is enabled.
     */
    public boolean isAutoAimEnabled() {
        if (DashboardHelper.getBoolean(Category.PRACTICE, "DisableAutoAim", false)) return false;
        if (!testModeEnabled) return true;
        return testAutoAimEnabled;
    }
    
    /**
     * Check if auto-shuttle is enabled.
     */
    public boolean isAutoShuttleEnabled() {
        if (DashboardHelper.getBoolean(Category.PRACTICE, "DisableAutoShuttle", false)) return false;
        if (!testModeEnabled) return true;
        return testAutoShuttleEnabled;
    }
    
    // --- Manual Control Checks ---
    
    /**
     * Check if manual shooter control is enabled.
     */
    public boolean isManualShooterControl() {
        return testManualShooterControl;
    }
    
    /**
     * Get the manual shooter RPM setpoint.
     */
    public double getManualShooterRPM() {
        return testShooterRPM;
    }
    
    /**
     * Check if manual turret control is enabled.
     */
    public boolean isManualTurretControl() {
        return testManualTurretControl;
    }
    
    /**
     * Get the manual turret angle setpoint.
     */
    public double getManualTurretAngle() {
        return testTurretAngle;
    }
    
    // --- Compound Checks (for easy subsystem use) ---
    
    /**
     * Check if the shooter should be allowed to run.
     * Considers all practice and test mode settings.
     */
    public boolean shouldShooterRun() {
        // Check practice mode disables
        if (DashboardHelper.getBoolean(Category.PRACTICE, "DisableShooter", false)) return false;
        if (practiceDriveOnly || practiceTurretOnly || practiceVisionOnly) return false;
        
        // Check test mode
        if (testModeEnabled && !testShooterEnabled) return false;
        
        return true;
    }
    
    /**
     * Check if the turret should be allowed to run.
     * Considers all practice and test mode settings.
     */
    public boolean shouldTurretRun() {
        // Check practice mode disables
        if (DashboardHelper.getBoolean(Category.PRACTICE, "DisableTurret", false)) return false;
        if (practiceDriveOnly || practiceShooterOnly || practiceVisionOnly) return false;
        
        // Check test mode
        if (testModeEnabled && !testTurretEnabled) return false;
        
        return true;
    }
    
    /**
     * Check if the drive should be allowed to run.
     * Considers all practice and test mode settings.
     */
    public boolean shouldDriveRun() {
        // Check practice mode disables
        if (DashboardHelper.getBoolean(Category.PRACTICE, "DisableDrive", false)) return false;
        if (practiceShooterOnly || practiceTurretOnly || practiceVisionOnly) return false;
        
        // Check test mode
        if (testModeEnabled && !testDriveEnabled) return false;
        
        return true;
    }
    
    /**
     * Get the effective speed multiplier considering all practice settings.
     * @return Speed multiplier from 0.0 to 1.0
     */
    public double getEffectiveSpeedMultiplier() {
        if (practiceSlowMotion) {
            return Math.min(0.5, practiceSpeedLimit);
        }
        return practiceSpeedLimit;
    }
    
    /**
     * Reset all game state (call at start of new match).
     */
    public void reset() {
        receivedGameMessage = false;
        firstActiveAlliance = null;
        forceShootEnabled = false;
        shuttleMode = false;
        shuttleModeManualOverride = false;
        inShuttleZone = false;
        practiceMatchMode = false;
        currentPhase = GamePhase.PRE_MATCH;
        currentTargetMode = TargetMode.DISABLED;
    }

    /**
     * Enable or disable practice match mode.
     * When enabled, uses alliance timing logic even without FMS.
     * @param enabled true to enable practice match mode
     */
    public void setPracticeMatchMode(boolean enabled) {
        this.practiceMatchMode = enabled;
        if (enabled) {
            System.out.println("=== PRACTICE MATCH MODE ENABLED ===");
        } else {
            System.out.println("=== PRACTICE MATCH MODE DISABLED ===");
        }
    }

    /**
     * Set which alliance goes first in practice match mode.
     * @param alliance The alliance that should be active first (Blue or Red)
     */
    public void setPracticeFirstAlliance(Alliance alliance) {
        this.firstActiveAlliance = alliance;
        this.receivedGameMessage = true;
        System.out.println("=== PRACTICE: " + alliance + " alliance set as first active ===");
    }
    
    // TIME REMAINING HELPERS
    
    /**
     * Get time remaining until our next active window.
     * Useful for driver awareness.
     * @return Seconds until our alliance is active (0 if already active)
     */
    public double getTimeUntilActive() {
        if (isOurAllianceActive()) {
            return 0;
        }
        
        double matchTime = DriverStation.getMatchTime();
        
        // Calculate time until next phase boundary where our alliance becomes active
        // Timer counts DOWN, so subtract the boundary time
        switch (currentPhase) {
            case SHIFT_1: return Math.max(0, matchTime - 105);  // Until Shift 2 at 1:45
            case SHIFT_2: return Math.max(0, matchTime - 80);   // Until Shift 3 at 1:20
            case SHIFT_3: return Math.max(0, matchTime - 55);   // Until Shift 4 at 0:55
            case SHIFT_4: return Math.max(0, matchTime - 30);   // Until End Game at 0:30
            default: return 0;
        }
    }
    
    /**
     * Get time remaining in current active window.
     * Useful for knowing how long we have to score.
     * @return Seconds remaining in current active window (0 if not active)
     */
    public double getTimeRemainingActive() {
        if (!isOurAllianceActive()) {
            return 0;
        }
        
        double matchTime = DriverStation.getMatchTime();
        
        // Calculate time until current phase ends (use Math.max to avoid negative values)
        switch (currentPhase) {
            case AUTO: return Math.max(0, matchTime);           // Until auto ends
            case TRANSITION: return Math.max(0, matchTime - 130); // Until Shift 1 at 2:10
            case SHIFT_1: return Math.max(0, matchTime - 105);  // Until 1:45
            case SHIFT_2: return Math.max(0, matchTime - 80);   // Until 1:20
            case SHIFT_3: return Math.max(0, matchTime - 55);   // Until 0:55
            case SHIFT_4: return Math.max(0, matchTime - 30);   // Until 0:30
            case END_GAME: return Math.max(0, matchTime);       // Until match ends
            default: return 0;
        }
    }

    /**
     * Get seconds until our alliance's NEXT shift starts.
     * This looks ahead to the next shift where WE are active, even if we're currently inactive.
     * Returns 0 if we're already in our active shift or endgame.
     * 
     * @return Seconds until our next active shift boundary (0 if already active or endgame)
     */
    public double getSecondsUntilOurNextShift() {
        if (!receivedGameMessage || firstActiveAlliance == null) {
            return 0;
        }
        
        double matchTime = DriverStation.getMatchTime();
        boolean weAreFirst = (robotAlliance == firstActiveAlliance);
        
        // Our shifts: if we are firstActiveAlliance -> SHIFT_1, SHIFT_3
        //             if we are other alliance     -> SHIFT_2, SHIFT_4
        // We want: time until the START of our next shift (from the opponent's shift)
        
        switch (currentPhase) {
            case TRANSITION:
                // Transition ends at 2:10, then Shift 1 starts
                if (weAreFirst) {
                    return Math.max(0, matchTime - 130); // Until Shift 1 starts at 2:10
                } else {
                    return Math.max(0, matchTime - 105); // Until Shift 2 starts at 1:45
                }
            case SHIFT_1:
                // Shift 1 is firstAlliance. If we're the other, next is Shift 2 at 1:45
                if (!weAreFirst) {
                    return Math.max(0, matchTime - 105);
                }
                return 0; // We're active now
            case SHIFT_2:
                // Shift 2 is otherAlliance. If we're first, next is Shift 3 at 1:20
                if (weAreFirst) {
                    return Math.max(0, matchTime - 80);
                }
                return 0; // We're active now
            case SHIFT_3:
                // Shift 3 is firstAlliance. If we're the other, next is Shift 4 at 0:55
                if (!weAreFirst) {
                    return Math.max(0, matchTime - 55);
                }
                return 0; // We're active now
            case SHIFT_4:
                // Shift 4 is otherAlliance. If we're first, next is End Game at 0:30
                if (weAreFirst) {
                    return Math.max(0, matchTime - 30);
                }
                return 0; // We're active now
            case END_GAME:
                return 0; // Both alliances active
            default:
                return 0;
        }
    }

    /**
     * Check if we're in the "head back" warning window.
     * This is TRUE when we are 5-3 seconds before OUR alliance's next shift starts.
     * Drivers should start heading back to scoring position.
     * 
     * @return true if 5 > secondsUntilOurShift >= 3
     */
    public boolean isHeadBackWarning() {
        if (isOurAllianceActive()) {
            return false; // Already active, no warning needed
        }
        double secsUntil = getSecondsUntilOurNextShift();
        return secsUntil > 3.0 && secsUntil <= 5.0;
    }

    /**
     * Check if we're in the "green light" pre-shift window.
     * This is TRUE when we are 3-0 seconds before OUR alliance's next shift starts.
     * Robot should be allowed to start shooting (green light!).
     * 
     * @return true if 3 >= secondsUntilOurShift > 0 and we're NOT yet in our shift
     */
    public boolean isGreenLightPreShift() {
        if (isOurAllianceActive()) {
            return false; // Already fully active, not "pre-shift"
        }
        double secsUntil = getSecondsUntilOurNextShift();
        return secsUntil > 0.0 && secsUntil <= 3.0;
    }

    /**
     * Check if shooting should be allowed right now.
     * Includes green light pre-shift window (3 seconds before our shift).
     * @return true if target mode is not DISABLED, OR we're in green light pre-shift
     */
    public boolean canShoot() {
        // Green light pre-shift always allows shooting
        if (isGreenLightPreShift()) {
            return true;
        }
        return currentTargetMode != TargetMode.DISABLED;
    }
    
    // TELEMETRY
    
    /**
     * Publish all game state data to SmartDashboard for debugging.
     */
    @SuppressWarnings("unused")
    private void publishTelemetry() {
        // Match status
        String matchStatus = "";
        if (isPreMatch()) {
            matchStatus = "[PRE-MATCH]";
        } else if (isPostMatch()) {
            matchStatus = "[POST-MATCH]";
        } else if (DriverStation.isAutonomous()) {
            matchStatus = "[AUTO]";
        } else if (DriverStation.isTeleop()) {
            matchStatus = "[TELEOP]";
        } else {
            matchStatus = "[UNKNOWN]";
        }
        DashboardHelper.putString(Category.HOME, "MATCH STATUS", matchStatus);
        
        // Key indicators for drivers
        DashboardHelper.putBoolean(Category.HOME, "SHOOT UNLOCKED", canShoot());
        DashboardHelper.putBoolean(Category.HOME, "HEAD BACK", isHeadBackWarning());
        DashboardHelper.putBoolean(Category.HOME, "GREEN LIGHT", isGreenLightPreShift());
        DashboardHelper.putBoolean(Category.HOME, "SHUTTLE ZONE ACTIVE", inShuttleZone);
    }
}





