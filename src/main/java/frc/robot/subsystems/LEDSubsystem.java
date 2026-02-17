package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.GameStateManager.GamePhase;
import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;

/**
 * LED subsystem using CTRE CANdle - because robots should look cool!
 * 
 * This handles all the visual feedback for the drivers and pit crew:
 * - Pre-match diagnostics (are all systems go?)
 * - Alliance colors during the match
 * - Action feedback (shooting, aiming, intaking, etc.)
 * - Warning flashes for E-stops and endgame
 * - Victory celebration after matches! 
 * 
 * The LEDs follow a state machine pattern, with the main robot state
 * taking priority over action states (which are temporary overlays).
 */
public class LEDSubsystem extends SubsystemBase {
    
    // Hardware
    private final CANdle candle;
    private final CANdleConfiguration candleConfig;

    // Animation control requests (reusable)
    private final SolidColor solidColorRequest;
    
    // Per-LED color buffer for addressable animations
    // Each LED has [R, G, B] values
    private final int[][] ledBuffer;
    private final int ledCount;
    
    // Master brightness control
    private double masterBrightness = 1.0;
    
    // Test mode for animations
    private boolean testMode = false;
    private boolean lastTestMode = false; // Track previous state to detect transitions
    
    // CANdle boot readiness -- hardware isn't responsive until fully booted
    private boolean candleReady = false;
    @SuppressWarnings("unused")
    private boolean configApplied = false;
    private double bootStartTime = 0;
    private static final double CANDLE_MIN_BOOT_SECONDS = 1.5; // Minimum wait before checking
    
    // Current pattern name for dashboard visibility
    private String currentPatternName = "Waiting for CANdle...";
    
    // State tracking
    private LEDState currentState = LEDState.BOOT_WARMUP;
    private ActionState currentAction = ActionState.IDLE;
    private double stateStartTime = 0;
    private int[] allianceColor = Constants.LEDs.BLUE_ALLIANCE;
    private boolean isEStopped = false;
    private boolean matchWasActive = false;  // Track if we were in a match
    private double matchEndCelebrationDuration = 10.0;  // Celebrate for 10 seconds after match
    
    // Progress/fill animation tracking (for spooling)
    private int fillLevel = 0;
    private double lastFillUpdate = 0;
    
    // Targeting animation tracking (for aiming - edges closing in)
    private int targetingPosition = 0;
    private double lastTargetingUpdate = 0;
    private boolean targetingClosing = true;
    
    // Endgame urgency tracking
    private int endgameFlashCount = 0;
    private double lastEndgameFlash = 0;
    
    // First active alliance flash tracking
    private boolean hasShownFirstAllianceFlash = false;
    private boolean isShowingFirstAllianceFlash = false;
    private int firstAllianceFlashCount = 0;
    private double lastFirstAllianceFlash = 0;
    private int[] firstActiveAllianceColor = null;
    
    // Game state manager for shift timing
    private final GameStateManager gameState;
    
    // Notification state tracking (to avoid spam)
    private boolean hasNotifiedEStop = false;
    private boolean hasNotifiedEndgame = false;
    
    /**
     * LED states for the pre-match sequence and robot operation
     */
    public enum LEDState {
        BOOT_WARMUP,        // Team colors animation - CANdle warming up
        NO_AUTO,            // Orange strobe - no auto routine selected
        FMS_WAIT,           // Yellow strobe - waiting for FMS connection
        ALL_SYSTEMS_GO,     // Green + team colors chase - ready to compete!
        DISABLED,           // Team colors chase - robot disabled
        AUTO,               // Fast alliance color chase
        TELEOP,             // Breathing alliance color
        ENDGAME,            // Rapid strobe alliance color
        MATCH_END,          // Post-match team color celebration!
        BROWNOUT            // Brown flash - brownout condition
    }
    
    /**
     * Robot action states for LED feedback during operation
     */
    public enum ActionState {
        IDLE,           // Robot doing nothing special
        SHOOTING,       // Actively shooting
        AIMING,         // Aiming turret at target
        SPOOLING,       // Spinning up shooter wheels
        DRIVING,        // Just driving (lowest priority)
        INTAKING,       // Running intake
        CLIMBING,       // Running climber
        ESTOP ,          // Emergency stop active
        BROWNOUT
    }
    
    /**
     * Creates and configures the LED subsystem.
     */
    public LEDSubsystem() {
        candle = new CANdle(Constants.CANIds.CANDLE);
        solidColorRequest = new SolidColor(0, Constants.LEDs.LED_COUNT - 1);
        gameState = GameStateManager.getInstance();
        
        // Initialize per-LED buffer for addressable animations
        ledCount = Constants.LEDs.LED_COUNT;
        ledBuffer = new int[ledCount][3];  // RGB for each LED
        
        // Build the CANdle config but DON'T apply yet --
        // the CANdle hardware isn't responsive until ~1-2s after power-on.
        // We'll apply it once the device is alive on the CAN bus.
        candleConfig = new CANdleConfiguration();
        candleConfig.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
        candleConfig.FutureProofConfigs = true;
        candleConfig.LED.StripType = StripTypeValue.GRB;  // Standard addressable RGB (ARGB) LEDs
        
        bootStartTime = Timer.getFPGATimestamp();
        stateStartTime = bootStartTime;
        
        // Initialize SmartDashboard controls
        initializeSmartDashboard();
    }
    
    /**
     * Initialize SmartDashboard controls for testing and brightness control.
     */
    private void initializeSmartDashboard() {
        DashboardHelper.putNumber(Category.DEBUG, "LED/Master_Brightness", masterBrightness);
        DashboardHelper.putBoolean(Category.DEBUG, "LED/Test_Mode", testMode);
    }
    
    /**
     * Sets the LED state.
     * Automatically detects match end and triggers celebration!
     */
    public void setState(LEDState newState) {
        if (currentState != newState) {
            // Check if we just finished a match (going from active play to disabled)
            boolean wasInMatch = (currentState == LEDState.AUTO || currentState == LEDState.TELEOP || currentState == LEDState.ENDGAME);
            boolean goingToDisabled = (newState == LEDState.DISABLED);
            
            // If we were in a match and now going disabled, celebrate!
            if (wasInMatch && goingToDisabled && matchWasActive) {
                newState = LEDState.MATCH_END;  // Override to celebration mode!
                matchWasActive = false;  // Reset so we don't celebrate again
            }
            
            // Track when match becomes active
            if (newState == LEDState.AUTO || newState == LEDState.TELEOP) {
                matchWasActive = true;
            }
            
            currentState = newState;
            stateStartTime = Timer.getFPGATimestamp();
        }
    }
    
    /**
     * Gets the current LED state.
     */
    public LEDState getState() {
        return currentState;
    }
    /**
     * Sets the current robot action for LED feedback.
     * Higher priority actions override lower priority ones.
     */
    public void setAction(ActionState action) {
        currentAction = action;
    }
    
    /**
     * Gets the current action state.
     */
    public ActionState getAction() {
        return currentAction;
    }
    
    /**
     * Clears the action state back to idle.
     */
    public void clearAction() {
        currentAction = ActionState.IDLE;
    }
    
    /**
     * Updates alliance color based on DriverStation.
     * Also checks if we just received the first active alliance info!
     */
    private void updateAllianceColor() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                allianceColor = Constants.LEDs.RED_ALLIANCE;
            } else {
                allianceColor = Constants.LEDs.BLUE_ALLIANCE;
            }
        }
        
        // Check if we just received the first active alliance from FMS!
        checkFirstActiveAllianceFlash();
    }
    
    /**
     * Checks if we've received the first active alliance info from FMS.
     * If we have, and haven't shown the flash yet, start the flash sequence!
     * This alerts drivers which alliance goes first in the match.
     */
    private void checkFirstActiveAllianceFlash() {
        // Only check if we haven't already shown the flash
        if (hasShownFirstAllianceFlash) {
            return;
        }
        
        // Check if game message has been received
        if (!gameState.hasReceivedGameMessage()) {
            return;
        }
        
        // Get the first active alliance
        Alliance firstAlliance = gameState.getFirstActiveAlliance();
        if (firstAlliance == null) {
            return;
        }
        
        // We got the message! Set up the flash
        if (firstAlliance == Alliance.Red) {
            firstActiveAllianceColor = Constants.LEDs.RED_ALLIANCE;
        } else {
            firstActiveAllianceColor = Constants.LEDs.BLUE_ALLIANCE;
        }
        
        // Start the flash sequence!
        isShowingFirstAllianceFlash = true;
        firstAllianceFlashCount = 10;  // 5 flashes = 10 states (on, off, on, off, on, off, on, off, on, off)
        lastFirstAllianceFlash = Timer.getFPGATimestamp();
        
        // Send notification to drivers!
        String allianceName = (firstAlliance == Alliance.Red) ? "RED" : "BLUE";
        
        Elastic.sendNotification(new Elastic.Notification()
            .withLevel(NotificationLevel.INFO)
            .withTitle(" FIRST ACTIVE: " + allianceName )
            .withDescription(allianceName + " alliance goes first in Shift 1!")
            .withDisplaySeconds(5.0));
        
        hasShownFirstAllianceFlash = true;  // Mark that we've handled this
    }
    
    /**
     * Updates the first active alliance flash animation.
     * Returns true if we're still flashing (override other patterns).
     */
    private boolean updateFirstAllianceFlash() {
        if (!isShowingFirstAllianceFlash || firstActiveAllianceColor == null) {
            return false;
        }
        
        double currentTime = Timer.getFPGATimestamp();
        double flashInterval = 0.15;  // Fast flash (150ms per state)
        
        if (currentTime - lastFirstAllianceFlash > flashInterval) {
            lastFirstAllianceFlash = currentTime;
            firstAllianceFlashCount--;
            
            if (firstAllianceFlashCount <= 0) {
                // Done flashing
                isShowingFirstAllianceFlash = false;
                return false;
            }
        }
        
        // Alternate between alliance color and off
        if (firstAllianceFlashCount % 2 == 1) {
            // ON - show the first active alliance color
            setSolidColor(firstActiveAllianceColor);
        } else {
            // OFF - turn off LEDs
            setSolidColor(Constants.LEDs.OFF);
        }
        
        return true;  // Still flashing, override other patterns
    }
    
    /**
     * Checks if emergency stop is active.
     */
    private void checkEStop() {
        isEStopped = DriverStation.isEStopped();
    }
    
    /**
     * Gets the total duration of a shift phase.
     */
    private double getShiftDuration(GamePhase phase) {
        switch (phase) {
            case SHIFT_1: return 25.0;
            case SHIFT_2: return 25.0;
            case SHIFT_3: return 25.0;
            case SHIFT_4: return 25.0;
            default: return 0.0;
        }
    }
    
    /**
     * Sets all LEDs to a solid color using Phoenix6 API.
     * This is the core method - all animations use this!
     * Colors are RGB format {Red, Green, Blue}
     */
    private void setSolidColor(int[] color) {
        int r = (int)(color[0] * masterBrightness);
        int g = (int)(color[1] * masterBrightness);
        int b = (int)(color[2] * masterBrightness);
        RGBWColor rgbw = new RGBWColor(r, g, b, 0);
        candle.setControl(solidColorRequest.withColor(rgbw));
    }
    
    // ===========================================
    // PER-LED ADDRESSABLE CONTROL METHODS
    // These methods control individual LEDs for real animations!
    // ===========================================
    
    /**
     * Sets a single LED in the buffer (doesn't push to hardware yet).
     */
    private void setLED(int index, int[] color) {
        if (index >= 0 && index < ledCount) {
            ledBuffer[index][0] = (int)(color[0] * masterBrightness);
            ledBuffer[index][1] = (int)(color[1] * masterBrightness);
            ledBuffer[index][2] = (int)(color[2] * masterBrightness);
        }
    }
    
    /**
     * Sets a single LED in the buffer with explicit RGB values.
     */
    private void setLED(int index, int r, int g, int b) {
        if (index >= 0 && index < ledCount) {
            ledBuffer[index][0] = (int)(r * masterBrightness);
            ledBuffer[index][1] = (int)(g * masterBrightness);
            ledBuffer[index][2] = (int)(b * masterBrightness);
        }
    }
    
    /**
     * Sets a range of LEDs to a color.
     */
    private void setLEDRange(int startIndex, int endIndex, int[] color) {
        for (int i = startIndex; i <= endIndex && i < ledCount; i++) {
            if (i >= 0) {
                setLED(i, color);
            }
        }
    }
    
    /**
     * Clears the LED buffer (all off).
     */
    private void clearBuffer() {
        for (int i = 0; i < ledCount; i++) {
            ledBuffer[i][0] = 0;
            ledBuffer[i][1] = 0;
            ledBuffer[i][2] = 0;
        }
    }
    
    /**
     * Sets the onboard CANdle LEDs (0-7) to a solid color in the buffer.
     * Call this after clearBuffer() and before pushBuffer() to give the
     * onboard LEDs a clean, separate look from the strip animation.
     */
    private void setOnboardColor(int[] color) {
        int onboard = Constants.LEDs.ONBOARD_LED_COUNT;
        for (int i = 0; i < onboard && i < ledCount; i++) {
            ledBuffer[i][0] = (int)(color[0] * masterBrightness);
            ledBuffer[i][1] = (int)(color[1] * masterBrightness);
            ledBuffer[i][2] = (int)(color[2] * masterBrightness);
        }
    }
    
    /**
     * Sets the onboard CANdle LEDs (0-7) to a breathing color in the buffer.
     * Smoothly pulses between 40% and 100% brightness on a 4-second cycle.
     */
    private void setOnboardBreathing(int[] color) {
        double currentTime = Timer.getFPGATimestamp();
        double breathPhase = (currentTime % 4.0) / 4.0;
        double breathBrightness = 0.4 + 0.6 * ((Math.sin(breathPhase * Math.PI * 2.0) + 1.0) / 2.0);
        
        int onboard = Constants.LEDs.ONBOARD_LED_COUNT;
        for (int i = 0; i < onboard && i < ledCount; i++) {
            ledBuffer[i][0] = (int)(color[0] * breathBrightness * masterBrightness);
            ledBuffer[i][1] = (int)(color[1] * breathBrightness * masterBrightness);
            ledBuffer[i][2] = (int)(color[2] * breathBrightness * masterBrightness);
        }
    }
    
    /**
     * Fills the entire buffer with one color.
     */
    private void fillBuffer(int[] color) {
        for (int i = 0; i < ledCount; i++) {
            setLED(i, color);
        }
    }
    
    /**
     * Pushes the LED buffer to hardware.
     * Uses efficient segment-based updates - groups consecutive LEDs with same color.
     */
    private void pushBuffer() {
        int i = 0;
        while (i < ledCount) {
            int startIdx = i;
            int r = ledBuffer[i][0];
            int g = ledBuffer[i][1];
            int b = ledBuffer[i][2];
            
            // Find consecutive LEDs with same color
            while (i < ledCount && 
                   ledBuffer[i][0] == r && 
                   ledBuffer[i][1] == g && 
                   ledBuffer[i][2] == b) {
                i++;
            }
            
            // Push this segment
            RGBWColor rgbw = new RGBWColor(r, g, b, 0);
            candle.setControl(new SolidColor(startIdx, i - 1).withColor(rgbw));
        }
    }
    
    // ===========================================
    // SIMPLE TIME-BASED ANIMATIONS
    // All animations just use setSolidColor with timing!
    // ===========================================
    
    /**
     * Breathing/pulsing effect - fades in and out.
     * Uses sine wave for smooth brightness modulation.
     * Colors are RGB format {Red, Green, Blue}
     */
    private void setBreathing(int[] color) {
        double currentTime = Timer.getFPGATimestamp();
        // 2-second breathing cycle
        double breathPhase = (currentTime % 2.0) / 2.0;
        double breathBrightness = (Math.sin(breathPhase * Math.PI * 2) + 1) / 2;
        
        int r = (int)(color[0] * masterBrightness * breathBrightness);
        int g = (int)(color[1] * masterBrightness * breathBrightness);
        int b = (int)(color[2] * masterBrightness * breathBrightness);
        
        RGBWColor rgbw = new RGBWColor(r, g, b, 0);
        candle.setControl(solidColorRequest.withColor(rgbw));
    }
    
    /**
     * Strobe effect - rapid on/off flashing.
     */
    private void setStrobe(int[] color) {
        double currentTime = Timer.getFPGATimestamp();
        // 10Hz strobe (on for 0.05s, off for 0.05s)
        boolean isOn = ((int)(currentTime * 10) % 2) == 0;
        
        if (isOn) {
            setSolidColor(color);
        } else {
            setSolidColor(Constants.LEDs.OFF);
        }
    }
    
    /**
     * Fast strobe effect - even more rapid flashing.
     */
    private void setFastStrobe(int[] color) {
        double currentTime = Timer.getFPGATimestamp();
        // 20Hz strobe
        boolean isOn = ((int)(currentTime * 20) % 2) == 0;
        
        if (isOn) {
            setSolidColor(color);
        } else {
            setSolidColor(Constants.LEDs.OFF);
        }
    }
    
    /**
     * Alternating two-color strobe.
     */
    private void setAlternatingStrobe(int[] color1, int[] color2, double speed) {
        double currentTime = Timer.getFPGATimestamp();
        boolean showFirst = ((int)(currentTime / speed) % 2) == 0;
        
        setSolidColor(showFirst ? color1 : color2);
    }
    
    /**
     * Chase animation - moving segments of two colors.
     * True addressable chase effect!
     */
    private void setChase(int[] color1, int[] color2) {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int segmentSize = 6;  // Size of each color segment
        int chasePos = (int)((currentTime / 1) % (segmentSize * 2));
        
        clearBuffer();
        setOnboardColor(color1);  // Onboard LEDs match the primary color
        
        for (int i = 0; i < stripCount; i++) {
            boolean isColor1 = ((i + chasePos) % (segmentSize * 2)) < segmentSize;
            setLED(stripStart + i, isColor1 ? color1 : color2);
        }
        
        pushBuffer();
    }
    
    /**
     * Team color chase pattern - dual comets flowing in opposite directions.
     * Orange comet goes one way, blue comet goes the other.
     * Looks polished and shows off team identity.
     */
    private void setTeamColorFlow() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int trailLength = 15;
        
        clearBuffer();
        
        // Onboard: split orange/blue
        int onboard = Constants.LEDs.ONBOARD_LED_COUNT;
        for (int i = 0; i < onboard && i < ledCount; i++) {
            setLED(i, (i < onboard / 2) ? Constants.LEDs.TEAM_SAFETY_ORANGE : Constants.LEDs.TEAM_BLUE);
        }
        
        // Orange comet going forward
        double phaseA = (currentTime % 5.0) / 5.0;
        int headA = (int)(phaseA * stripCount) % stripCount;
        int[] orange = Constants.LEDs.TEAM_SAFETY_ORANGE;
        
        for (int t = 0; t < trailLength; t++) {
            int idx = ((headA - t) % stripCount + stripCount) % stripCount;
            double fade = 1.0 - ((double) t / trailLength);
            fade = fade * fade;
            ledBuffer[stripStart + idx][0] = Math.max(ledBuffer[stripStart + idx][0], (int)(orange[0] * fade * masterBrightness));
            ledBuffer[stripStart + idx][1] = Math.max(ledBuffer[stripStart + idx][1], (int)(orange[1] * fade * masterBrightness));
            ledBuffer[stripStart + idx][2] = Math.max(ledBuffer[stripStart + idx][2], (int)(orange[2] * fade * masterBrightness));
        }
        
        // Blue comet going backward (opposite direction)
        double phaseB = (1.0 - (currentTime % 5.0) / 5.0) % 1.0;
        int headB = (int)(phaseB * stripCount) % stripCount;
        int[] blue = Constants.LEDs.TEAM_BLUE;
        
        for (int t = 0; t < trailLength; t++) {
            int idx = ((headB + t) % stripCount + stripCount) % stripCount;
            double fade = 1.0 - ((double) t / trailLength);
            fade = fade * fade;
            ledBuffer[stripStart + idx][0] = Math.max(ledBuffer[stripStart + idx][0], (int)(blue[0] * fade * masterBrightness));
            ledBuffer[stripStart + idx][1] = Math.max(ledBuffer[stripStart + idx][1], (int)(blue[1] * fade * masterBrightness));
            ledBuffer[stripStart + idx][2] = Math.max(ledBuffer[stripStart + idx][2], (int)(blue[2] * fade * masterBrightness));
        }
        
        pushBuffer();
    }
    
    /**
     * Victory celebration - sparkling fireworks with team colors!
     * Random twinkles of orange, blue, and white pop and fade across the strip
     * like fireworks exploding. Onboard LEDs flash white rapidly.
     * Way more exciting than spinning blocks!
     */
    private void setVictoryCelebration() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        int[] orange = Constants.LEDs.TEAM_SAFETY_ORANGE;
        int[] blue = Constants.LEDs.TEAM_BLUE;
        int[] white = Constants.LEDs.WHITE;
        
        clearBuffer();
        
        // Onboard LEDs: rapid white/orange/blue cycle
        int onboardCycle = ((int)(currentTime * 12)) % 3;
        int[] onboardColor = (onboardCycle == 0) ? white : (onboardCycle == 1) ? orange : blue;
        setOnboardColor(onboardColor);
        
        // Sparkling fireworks on strip
        // Use time-seeded pseudo-random to create consistent sparkle positions
        // that change over time
        for (int i = 0; i < stripCount; i++) {
            // Each LED has its own sparkle timing based on position
            // Use golden ratio offset to prevent visible patterns
            double ledSeed = (i * 0.618033988749895 + currentTime * 3.0) % 1.0;
            double sparkleChance = Math.sin(ledSeed * Math.PI * 2.0 + currentTime * 7.0 + i * 2.3);
            
            if (sparkleChance > 0.7) {
                // This LED is sparkling! Pick a color based on position
                double colorSeed = Math.sin(i * 1.7 + currentTime * 0.5);
                int[] sparkColor;
                double intensity = (sparkleChance - 0.7) / 0.3;  // 0 to 1
                intensity = intensity * intensity;  // More dramatic pop
                
                if (colorSeed > 0.3) {
                    sparkColor = orange;
                } else if (colorSeed > -0.3) {
                    sparkColor = white;
                } else {
                    sparkColor = blue;
                }
                
                ledBuffer[stripStart + i][0] = (int)(sparkColor[0] * intensity * masterBrightness);
                ledBuffer[stripStart + i][1] = (int)(sparkColor[1] * intensity * masterBrightness);
                ledBuffer[stripStart + i][2] = (int)(sparkColor[2] * intensity * masterBrightness);
            }
            // LEDs not sparkling stay dark -- makes the sparkles pop!
        }
        
        pushBuffer();
    }
    
    // ===========================================
    // CREATIVE INFORMATIVE PATTERNS
    // These patterns are designed to be INSTANTLY recognizable
    // and look GREAT for the audience!
    // ===========================================
    
    /**
     * E-STOP PATTERN - Dramatic red shockwaves pulsing from center outward.
     * Two red rings expand from the center simultaneously, with a deep red
     * background and bright white flash at the edges. Onboard LEDs solid red.
     * Unmistakable "DANGER - STOPPED" but looks incredible.
     */
    private void setEStopPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;
        
        clearBuffer();
        
        // Onboard LEDs: fast red strobe for urgency
        boolean onboardOn = ((int)(currentTime * 8) % 2) == 0;
        setOnboardColor(onboardOn ? Constants.LEDs.ESTOP_COLOR : Constants.LEDs.OFF);
        
        // Deep red base on entire strip
        int[] deepRed = {30, 0, 0};
        for (int i = 0; i < stripCount; i++) {
            setLED(stripStart + i, deepRed);
        }
        
        // Two shockwave rings expanding from center (1-second cycle)
        double ringPhase = (currentTime * 1.5) % 1.0;  // 1.5 rings per second
        int ringPos = (int)(ringPhase * center);
        
        // Second ring offset by half a cycle
        double ring2Phase = (currentTime * 1.5 + 0.5) % 1.0;
        int ring2Pos = (int)(ring2Phase * center);
        
        // Draw ring 1 (bright red with white leading edge)
        for (int i = 0; i < 6; i++) {
            int pos = ringPos - i;
            if (pos >= 0 && pos < center) {
                int brightness = 255 - (i * 40);
                int[] trailColor;
                if (i == 0) {
                    trailColor = new int[]{255, 80, 40};  // Hot orange-red leading edge
                } else {
                    trailColor = new int[]{brightness, 0, 0};
                }
                setLED(stripStart + center - pos, trailColor);
                setLED(stripStart + center + pos, trailColor);
            }
        }
        
        // Draw ring 2 
        for (int i = 0; i < 6; i++) {
            int pos = ring2Pos - i;
            if (pos >= 0 && pos < center) {
                int brightness = 255 - (i * 40);
                int[] trailColor;
                if (i == 0) {
                    trailColor = new int[]{255, 80, 40};
                } else {
                    trailColor = new int[]{brightness, 0, 0};
                }
                // Only set if brighter than existing
                int leftIdx = stripStart + center - pos;
                int rightIdx = stripStart + center + pos;
                if (leftIdx >= stripStart && ledBuffer[leftIdx][0] < brightness) {
                    setLED(leftIdx, trailColor);
                }
                if (rightIdx < stripStart + stripCount && ledBuffer[rightIdx][0] < brightness) {
                    setLED(rightIdx, trailColor);
                }
            }
        }
        
        // Bright pulsing center
        double centerPulse = (Math.sin(currentTime * Math.PI * 6) + 1) / 2;
        int[] centerColor = {255, (int)(40 * centerPulse), (int)(20 * centerPulse)};
        setLED(stripStart + center - 1, centerColor);
        setLED(stripStart + center, centerColor);
        
        pushBuffer();
    }
    
    /**
     * TARGETING PATTERN - "Edges closing in" with pulsing center crosshair.
     * Yellow brackets sweep inward from both ends like a targeting reticle.
     * A pulsing dim yellow glow at the center acts as the crosshair target.
     * When brackets meet, the whole strip flashes bright -- "LOCKED ON!"
     * Instantly says: "I'm locking onto something!"
     */
    private void setTargetingPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        // Update targeting animation at 20Hz
        if (currentTime - lastTargetingUpdate > 0.05) {
            lastTargetingUpdate = currentTime;
            
            if (targetingClosing) {
                targetingPosition++;
                if (targetingPosition >= stripCount / 2) {
                    targetingClosing = false;
                }
            } else {
                targetingPosition--;
                if (targetingPosition <= 0) {
                    targetingClosing = true;
                }
            }
        }
        
        int center = stripCount / 2;
        
        clearBuffer();
        
        // Onboard LEDs: pulsing yellow
        setOnboardBreathing(Constants.LEDs.AIMING_COLOR);
        
        // When brackets meet in center, flash bright to show "LOCKED"
        if (targetingPosition >= center - 2) {
            for (int i = 0; i < stripCount; i++) {
                setLED(stripStart + i, Constants.LEDs.AIMING_COLOR);
            }
        } else {
            // Pulsing crosshair glow at center (always visible as a target)
            double centerPulse = (Math.sin(currentTime * Math.PI * 4) + 1) / 2;
            int[] centerGlow = {(int)(100 * centerPulse), (int)(100 * centerPulse), 0};
            for (int c = -2; c <= 2; c++) {
                int idx = center + c;
                if (idx >= 0 && idx < stripCount) {
                    double falloff = 1.0 - Math.abs(c) * 0.3;
                    int[] glow = {(int)(centerGlow[0] * falloff), (int)(centerGlow[1] * falloff), 0};
                    setLED(stripStart + idx, glow);
                }
            }
            
            // Left bracket with bright head and fading trail
            for (int i = 0; i <= targetingPosition && i < stripCount; i++) {
                int distFromHead = targetingPosition - i;
                double fade = Math.max(0.15, 1.0 - distFromHead * 0.08);
                int[] bracketColor = {
                    (int)(Constants.LEDs.AIMING_COLOR[0] * fade),
                    (int)(Constants.LEDs.AIMING_COLOR[1] * fade),
                    0
                };
                setLED(stripStart + i, bracketColor);
            }
            
            // Right bracket with bright head and fading trail
            for (int i = stripCount - 1; i >= stripCount - 1 - targetingPosition && i >= 0; i--) {
                int distFromHead = i - (stripCount - 1 - targetingPosition);
                double fade = Math.max(0.15, 1.0 - distFromHead * 0.08);
                int[] bracketColor = {
                    (int)(Constants.LEDs.AIMING_COLOR[0] * fade),
                    (int)(Constants.LEDs.AIMING_COLOR[1] * fade),
                    0
                };
                setLED(stripStart + i, bracketColor);
            }
        }
        pushBuffer();
    }
    
    /**
     * POWER FILL PATTERN - Dramatic charging progress bar.
     * Orange fills from left to right with a bright white leading edge
     * and an energy shimmer through the filled section.
     * Onboard LEDs pulse faster as it fills up -- building tension!
     * Instantly says: "Building power, almost ready!"
     */
    private void setPowerFillPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        // Increment fill tracking at 10Hz
        if (currentTime - lastFillUpdate > 0.1) {
            lastFillUpdate = currentTime;
            fillLevel++;
            if (fillLevel > stripCount) {
                fillLevel = 0;
            }
        }
        
        clearBuffer();
        
        // Onboard LEDs: pulse speed increases with fill level
        double fillRatio = (double) fillLevel / stripCount;
        double pulseSpeed = 2.0 + fillRatio * 8.0;  // 2Hz to 10Hz as it charges
        double onboardPulse = (Math.sin(currentTime * Math.PI * pulseSpeed) + 1) / 2;
        int[] onboardColor = {
            (int)(Constants.LEDs.SPOOLING_COLOR[0] * (0.3 + 0.7 * onboardPulse) * masterBrightness),
            (int)(Constants.LEDs.SPOOLING_COLOR[1] * (0.3 + 0.7 * onboardPulse) * masterBrightness),
            (int)(Constants.LEDs.SPOOLING_COLOR[2] * (0.3 + 0.7 * onboardPulse) * masterBrightness)
        };
        int onboard = Constants.LEDs.ONBOARD_LED_COUNT;
        for (int i = 0; i < onboard && i < ledCount; i++) {
            ledBuffer[i][0] = onboardColor[0];
            ledBuffer[i][1] = onboardColor[1];
            ledBuffer[i][2] = onboardColor[2];
        }
        
        // Unfilled portion (very dim orange)
        int[] dimOrange = {20, 8, 0};
        for (int i = fillLevel + 1; i < stripCount; i++) {
            setLED(stripStart + i, dimOrange);
        }
        
        // Filled portion with energy shimmer
        for (int i = 0; i < fillLevel && i < stripCount; i++) {
            double shimmer = Math.sin(currentTime * 12.0 + i * 0.5) * 0.15 + 0.85;
            int[] shimmerColor = {
                (int)(Constants.LEDs.SPOOLING_COLOR[0] * shimmer),
                (int)(Constants.LEDs.SPOOLING_COLOR[1] * shimmer),
                (int)(Constants.LEDs.SPOOLING_COLOR[2] * shimmer)
            };
            setLED(stripStart + i, shimmerColor);
        }
        
        // Bright white-hot leading edge (3 LEDs)
        for (int e = 0; e < 3; e++) {
            int edgeIdx = fillLevel - e;
            if (edgeIdx >= 0 && edgeIdx < stripCount) {
                double edgeFade = 1.0 - e * 0.3;
                int[] edgeColor = {(int)(255 * edgeFade), (int)(200 * edgeFade), (int)(80 * edgeFade)};
                setLED(stripStart + edgeIdx, edgeColor);
            }
        }
        
        pushBuffer();
    }
    
    /**
     * INTAKE FLOW PATTERN - Dual streams being sucked toward center.
     * Two bright comets with long trails chase from both edges to center.
     * When they reach center, a bright flash pulses -- "NOM!"
     * Onboard LEDs breathe in sync.
     * Instantly says: "Eating a game piece!"
     */
    private void setIntakeFlowPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;
        
        // Chase position moving inward (0.6 second cycle)
        double flowPhase = (currentTime % 0.6) / 0.6;
        int chasePos = (int)(flowPhase * center);
        
        clearBuffer();
        setOnboardBreathing(Constants.LEDs.INTAKING_COLOR);
        
        int[] color = Constants.LEDs.INTAKING_COLOR;
        int trailLength = 8;
        
        // Left comet flowing right (toward center)
        for (int i = 0; i < trailLength; i++) {
            int pos = chasePos - i;
            if (pos >= 0 && pos < center) {
                double fade = 1.0 - ((double) i / trailLength);
                fade = fade * fade;
                int[] trailColor = {(int)(color[0] * fade), (int)(color[1] * fade), (int)(color[2] * fade)};
                setLED(stripStart + pos, trailColor);
            }
        }
        // White leading edge
        if (chasePos >= 0 && chasePos < center) {
            setLED(stripStart + chasePos, new int[]{255, 200, 220});
        }
        
        // Right comet flowing left (toward center)
        int rightChase = stripCount - 1 - chasePos;
        for (int i = 0; i < trailLength; i++) {
            int pos = rightChase + i;
            if (pos >= center && pos < stripCount) {
                double fade = 1.0 - ((double) i / trailLength);
                fade = fade * fade;
                int[] trailColor = {(int)(color[0] * fade), (int)(color[1] * fade), (int)(color[2] * fade)};
                setLED(stripStart + pos, trailColor);
            }
        }
        // White leading edge
        if (rightChase >= center && rightChase < stripCount) {
            setLED(stripStart + rightChase, new int[]{255, 200, 220});
        }
        
        // Center "gulp" glow -- brighter when comets are near center
        double nearCenter = 1.0 - (double) Math.abs(chasePos - center) / center;
        nearCenter = nearCenter * nearCenter * nearCenter;  // Cubic for dramatic pop
        int[] centerGlow = {(int)(color[0] * nearCenter), (int)(color[1] * nearCenter), (int)(color[2] * nearCenter)};
        for (int c = -2; c <= 2; c++) {
            int idx = center + c;
            if (idx >= 0 && idx < stripCount) {
                double spread = 1.0 - Math.abs(c) * 0.25;
                setLED(stripStart + idx, new int[]{(int)(centerGlow[0] * spread), (int)(centerGlow[1] * spread), (int)(centerGlow[2] * spread)});
            }
        }
        
        pushBuffer();
    }
    
    /**
     * CLIMB RISING PATTERN - Multiple purple waves rising upward.
     * Three evenly-spaced comets rise up the strip simultaneously
     * with trails that make it look like ascending energy.
     * Onboard LEDs pulse purple.
     * Instantly says: "Going UP!"
     */
    private void setClimbRisingPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        clearBuffer();
        setOnboardBreathing(Constants.LEDs.CLIMBING_COLOR);
        
        int[] color = Constants.LEDs.CLIMBING_COLOR;
        int trailLength = 10;
        int numComets = 3;
        
        // Rising position (loops, 2 second cycle)
        double risePhase = (currentTime % 2.0) / 2.0;
        
        for (int c = 0; c < numComets; c++) {
            double cometPhase = (risePhase + (double) c / numComets) % 1.0;
            int risePos = (int)(cometPhase * stripCount);
            
            for (int i = 0; i < trailLength; i++) {
                int pos = risePos - i;
                if (pos >= 0 && pos < stripCount) {
                    double fade = 1.0 - ((double) i / trailLength);
                    fade = fade * fade;
                    int r = Math.max(ledBuffer[stripStart + pos][0], (int)(color[0] * fade * masterBrightness));
                    int g = Math.max(ledBuffer[stripStart + pos][1], (int)(color[1] * fade * masterBrightness));
                    int b = Math.max(ledBuffer[stripStart + pos][2], (int)(color[2] * fade * masterBrightness));
                    ledBuffer[stripStart + pos][0] = r;
                    ledBuffer[stripStart + pos][1] = g;
                    ledBuffer[stripStart + pos][2] = b;
                }
            }
            // White tip on each comet
            if (risePos >= 0 && risePos < stripCount) {
                setLED(stripStart + risePos, new int[]{220, 150, 255});
            }
        }
        
        pushBuffer();
    }
    
    /**
     * READY SPLIT PATTERN - Half orange, half blue with breathing boundary.
     * Shows team identity and readiness simultaneously.
     * Instantly says: "We're Team 4539 and we're READY!"
     */
    private void setReadySplitPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;
        
        // Breathing effect on the boundary
        double breathPhase = (currentTime % 2.0) / 2.0;
        double breathBrightness = (Math.sin(breathPhase * Math.PI * 2) + 1) / 2;
        
        clearBuffer();
        setOnboardColor(Constants.LEDs.GREEN);  // Onboard LEDs green = READY
        
        // Left half - team orange
        setLEDRange(stripStart, stripStart + center - 1, Constants.LEDs.TEAM_SAFETY_ORANGE);
        
        // Right half - team blue
        setLEDRange(stripStart + center, stripStart + stripCount - 1, Constants.LEDs.TEAM_BLUE);
        
        // Breathing white boundary at center
        int whiteBrightness = (int)(255 * breathBrightness);
        int[] breathingWhite = {whiteBrightness, whiteBrightness, whiteBrightness};
        setLED(stripStart + center - 1, breathingWhite);
        setLED(stripStart + center, breathingWhite);
        
        pushBuffer();
    }
    
    /**
     * ENDGAME URGENCY PATTERN - Accelerating heartbeat with shockwaves.
     * Alliance-colored pulses radiate from center, getting faster over time.
     * Every few seconds a bright white flash grabs attention.
     * The "heartbeat" speeds up as match time decreases -- real urgency!
     * Instantly says: "HURRY! Time is almost up!"
     */
    private void setEndgameUrgencyPattern() {
        double currentTime = Timer.getFPGATimestamp();
        double matchTime = DriverStation.getMatchTime();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;
        
        // Every 3 seconds, do a quick white flash
        if (currentTime - lastEndgameFlash > 3.0) {
            lastEndgameFlash = currentTime;
            endgameFlashCount = 2;
        }
        
        clearBuffer();
        
        // Onboard: rapid alliance color strobe
        boolean onboardOn = ((int)(currentTime * 6) % 2) == 0;
        setOnboardColor(onboardOn ? allianceColor : Constants.LEDs.WHITE);
        
        if (endgameFlashCount > 0) {
            endgameFlashCount--;
            // White flash burst
            for (int i = 0; i < stripCount; i++) {
                setLED(stripStart + i, Constants.LEDs.WHITE);
            }
        } else {
            // Heartbeat speed -- faster as time runs out (1Hz to 4Hz)
            double heartRate = 1.0 + (1.0 - Math.max(0, Math.min(1.0, matchTime / 30.0))) * 3.0;
            double beatPhase = (currentTime * heartRate) % 1.0;
            
            // Heartbeat has two quick pulses (lub-dub)
            double beatIntensity;
            if (beatPhase < 0.1) {
                beatIntensity = beatPhase / 0.1;  // First beat rise
            } else if (beatPhase < 0.2) {
                beatIntensity = 1.0 - (beatPhase - 0.1) / 0.1;  // First beat fall
            } else if (beatPhase < 0.3) {
                beatIntensity = (beatPhase - 0.2) / 0.1 * 0.7;  // Second beat rise (smaller)
            } else if (beatPhase < 0.4) {
                beatIntensity = 0.7 * (1.0 - (beatPhase - 0.3) / 0.1);  // Second beat fall
            } else {
                beatIntensity = 0;  // Rest between beats
            }
            
            // Apply heartbeat as brightness to alliance color across strip
            // with a radial falloff from center
            for (int i = 0; i < stripCount; i++) {
                double distFromCenter = Math.abs(i - center) / (double) center;
                double radialFade = 1.0 - distFromCenter * 0.5;  // Center brighter than edges
                double finalIntensity = beatIntensity * radialFade;
                finalIntensity = Math.max(0.05, finalIntensity);  // Never fully dark
                
                ledBuffer[stripStart + i][0] = (int)(allianceColor[0] * finalIntensity * masterBrightness);
                ledBuffer[stripStart + i][1] = (int)(allianceColor[1] * finalIntensity * masterBrightness);
                ledBuffer[stripStart + i][2] = (int)(allianceColor[2] * finalIntensity * masterBrightness);
            }
        }
        pushBuffer();
    }
    
    /**
     * BROWNOUT PATTERN - Dramatic power failure / sputtering.
     * LEDs flicker and sections randomly go dark then struggle back on,
     * like a dying lightbulb. The brown/amber color screams "low power."
     * Onboard LEDs flicker weakly.
     * Instantly says: "Low power! Battery dying!"
     */
    private void setBrownoutPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        clearBuffer();
        
        // Onboard LEDs: dim flickering brown
        double onboardFlicker = Math.sin(currentTime * 15) * 0.3 + 0.4;
        onboardFlicker = Math.max(0.1, onboardFlicker);
        int[] brown = Constants.LEDs.BROWNOUT_COLOR;
        int onboard = Constants.LEDs.ONBOARD_LED_COUNT;
        for (int i = 0; i < onboard && i < ledCount; i++) {
            ledBuffer[i][0] = (int)(brown[0] * onboardFlicker * masterBrightness);
            ledBuffer[i][1] = (int)(brown[1] * onboardFlicker * masterBrightness);
            ledBuffer[i][2] = (int)(brown[2] * onboardFlicker * masterBrightness);
        }
        
        // Strip: sections randomly go dark and sputter back
        for (int i = 0; i < stripCount; i++) {
            // Each LED has its own "power stability" using layered sine waves
            double stability = Math.sin(currentTime * 13.7 + i * 2.1) 
                             * Math.sin(currentTime * 7.3 + i * 0.9)
                             * Math.sin(currentTime * 3.1 + i * 4.7);
            
            // Normalize from [-1,1] to [0,1] with a bias toward dim
            double brightness = (stability + 1.0) / 2.0;
            brightness = brightness * brightness;  // Square to make dark sections more common
            
            // Occasionally a LED "sparks" bright (power surge)
            double sparkChance = Math.sin(currentTime * 31.0 + i * 5.3);
            if (sparkChance > 0.95) {
                brightness = 1.0;  // Full bright spark!
            }
            
            // Minimum dim glow so it's not completely black
            brightness = Math.max(0.05, brightness);
            
            ledBuffer[stripStart + i][0] = (int)(brown[0] * brightness * masterBrightness);
            ledBuffer[stripStart + i][1] = (int)(brown[1] * brightness * masterBrightness);
            ledBuffer[stripStart + i][2] = (int)(brown[2] * brightness * masterBrightness);
        }
        
        pushBuffer();
    }
    
    /**
     * WAITING/DISABLED PATTERN - Clean team color idle animation.
     * 
     * Onboard LEDs (0-7): Gentle alternating orange/blue breathing glow.
     * Strip LEDs (8-67): Smooth comet tails -- an orange comet and a blue comet
     * chase each other around the strip on a dark background. Each comet is a
     * bright head with a fading trail behind it. The two comets are always on
     * opposite sides of the strip so they never overlap or mix into ugly colors.
     * 
     * Instantly says: "I'm here, waiting for you"
     */
    private void setWaitingPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int[] orange = Constants.LEDs.TEAM_SAFETY_ORANGE;
        int[] blue = Constants.LEDs.TEAM_BLUE;
        int onboard = Constants.LEDs.ONBOARD_LED_COUNT;
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        clearBuffer();
        
        // === ONBOARD LEDs (0-7): Gentle breathing team color split ===
        // Left 4 = orange, right 4 = blue, with synchronized breathing
        double breathPhase = (currentTime % 4.0) / 4.0;  // 4-second cycle
        double breathBrightness = 0.4 + 0.6 * ((Math.sin(breathPhase * Math.PI * 2.0) + 1.0) / 2.0);
        
        for (int i = 0; i < onboard && i < ledCount; i++) {
            int[] color = (i < onboard / 2) ? orange : blue;
            int r = (int)(color[0] * breathBrightness * masterBrightness);
            int g = (int)(color[1] * breathBrightness * masterBrightness);
            int b = (int)(color[2] * breathBrightness * masterBrightness);
            ledBuffer[i][0] = r;
            ledBuffer[i][1] = g;
            ledBuffer[i][2] = b;
        }
        
        // === STRIP LEDs (8-67): Dual comet tails chasing each other ===
        // Two comets on opposite sides of the strip, moving at a calm pace.
        // 8-second full loop = smooth and relaxed.
        double cometPhase = (currentTime % 8.0) / 8.0;
        int cometHeadA = (int)(cometPhase * stripCount) % stripCount;          // Orange comet
        int cometHeadB = (cometHeadA + stripCount / 2) % stripCount;           // Blue comet (opposite side)
        
        int trailLength = 26;  // How many LEDs the tail stretches
        
        // Draw orange comet (head + fading trail)
        for (int t = 0; t < trailLength; t++) {
            int stripIdx = ((cometHeadA - t) % stripCount + stripCount) % stripCount;
            int ledIdx = stripStart + stripIdx;
            
            // Brightness falls off quadratically for a nice tail shape
            double tailFactor = 1.0 - ((double) t / trailLength);
            tailFactor = tailFactor * tailFactor;  // Quadratic falloff
            
            if (ledIdx < ledCount) {
                ledBuffer[ledIdx][0] = (int)(orange[0] * tailFactor * masterBrightness);
                ledBuffer[ledIdx][1] = (int)(orange[1] * tailFactor * masterBrightness);
                ledBuffer[ledIdx][2] = (int)(orange[2] * tailFactor * masterBrightness);
            }
        }
        
        // Draw blue comet (head + fading trail)
        for (int t = 0; t < trailLength; t++) {
            int stripIdx = ((cometHeadB - t) % stripCount + stripCount) % stripCount;
            int ledIdx = stripStart + stripIdx;
            
            double tailFactor = 1.0 - ((double) t / trailLength);
            tailFactor = tailFactor * tailFactor;
            
            if (ledIdx < ledCount) {
                ledBuffer[ledIdx][0] = (int)(blue[0] * tailFactor * masterBrightness);
                ledBuffer[ledIdx][1] = (int)(blue[1] * tailFactor * masterBrightness);
                ledBuffer[ledIdx][2] = (int)(blue[2] * tailFactor * masterBrightness);
            }
        }
        
        pushBuffer();
    }
    
    /**
     * SHOOT NOW PATTERN - Green shockwave explosions from center.
     * Bright green rings blast outward from center with white-hot leading edges.
     * Multiple overlapping rings for continuous "FIRE!" feeling.
     * Onboard LEDs solid bright green.
     * Instantly says: "FIRE! SHOOT NOW!"
     */
    private void setShootNowPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;
        
        clearBuffer();
        setOnboardColor(Constants.LEDs.SHOOTING_COLOR);
        
        int[] color = Constants.LEDs.SHOOTING_COLOR;
        
        // Dim green base so strip is never fully dark
        for (int i = 0; i < stripCount; i++) {
            ledBuffer[stripStart + i][0] = (int)(8 * masterBrightness);
            ledBuffer[stripStart + i][1] = (int)(30 * masterBrightness);
            ledBuffer[stripStart + i][2] = (int)(8 * masterBrightness);
        }
        
        // Three overlapping rings with different speeds for continuous look
        int numRings = 3;
        for (int ring = 0; ring < numRings; ring++) {
            double ringPhase = (currentTime * 3.0 + ring * 0.33) % 1.0;
            int ringPos = (int)(ringPhase * center);
            int trailLength = 6;
            
            for (int t = 0; t < trailLength; t++) {
                int pos = ringPos - t;
                if (pos >= 0 && pos < center) {
                    double fade = 1.0 - ((double) t / trailLength);
                    fade = fade * fade;
                    
                    int[] trailColor;
                    if (t == 0) {
                        // White-hot leading edge
                        trailColor = new int[]{(int)(200 * fade), (int)(255 * fade), (int)(200 * fade)};
                    } else {
                        trailColor = new int[]{(int)(color[0] * fade), (int)(color[1] * fade), (int)(color[2] * fade)};
                    }
                    
                    // Left of center
                    int leftIdx = stripStart + center - pos;
                    if (leftIdx >= stripStart) {
                        ledBuffer[leftIdx][0] = Math.max(ledBuffer[leftIdx][0], (int)(trailColor[0] * masterBrightness));
                        ledBuffer[leftIdx][1] = Math.max(ledBuffer[leftIdx][1], (int)(trailColor[1] * masterBrightness));
                        ledBuffer[leftIdx][2] = Math.max(ledBuffer[leftIdx][2], (int)(trailColor[2] * masterBrightness));
                    }
                    // Right of center
                    int rightIdx = stripStart + center + pos;
                    if (rightIdx < stripStart + stripCount) {
                        ledBuffer[rightIdx][0] = Math.max(ledBuffer[rightIdx][0], (int)(trailColor[0] * masterBrightness));
                        ledBuffer[rightIdx][1] = Math.max(ledBuffer[rightIdx][1], (int)(trailColor[1] * masterBrightness));
                        ledBuffer[rightIdx][2] = Math.max(ledBuffer[rightIdx][2], (int)(trailColor[2] * masterBrightness));
                    }
                }
            }
        }
        
        // Bright pulsing center
        double centerPulse = (Math.sin(currentTime * Math.PI * 8) + 1) / 2 * 0.3 + 0.7;
        int[] brightCenter = {(int)(100 * centerPulse), (int)(255 * centerPulse), (int)(100 * centerPulse)};
        setLED(stripStart + center - 1, brightCenter);
        setLED(stripStart + center, brightCenter);
        
        pushBuffer();
    }
    
    /**
     * AUTO MODE PATTERN - Dual alliance-colored comets racing opposite directions.
     * Two bright comets circling the strip in opposite directions with white-hot tips.
     * Energetic and fast -- clearly distinct from teleop's calmer wave.
     * White sparkle accents randomly appear and fade for extra energy.
     * Onboard LEDs pulse alliance color at 2Hz.
     * Instantly says: "Robot is driving itself!"
     */
    private void setAutoModePattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        clearBuffer();
        // Onboard: fast alliance pulse to look "robotic"
        double onboardPulse = (Math.sin(currentTime * Math.PI * 4) + 1) / 2 * 0.6 + 0.4;
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT; i++) {
            ledBuffer[i][0] = (int)(allianceColor[0] * onboardPulse * masterBrightness);
            ledBuffer[i][1] = (int)(allianceColor[1] * onboardPulse * masterBrightness);
            ledBuffer[i][2] = (int)(allianceColor[2] * onboardPulse * masterBrightness);
        }
        
        // Very dim base so the strip isn't dead black
        for (int i = 0; i < stripCount; i++) {
            ledBuffer[stripStart + i][0] = (int)(allianceColor[0] * 0.04 * masterBrightness);
            ledBuffer[stripStart + i][1] = (int)(allianceColor[1] * 0.04 * masterBrightness);
            ledBuffer[stripStart + i][2] = (int)(allianceColor[2] * 0.04 * masterBrightness);
        }
        
        // Comet 1: forward direction, 3-second loop
        double speed1 = currentTime * 20.0;
        int cometPos1 = ((int) speed1 % stripCount + stripCount) % stripCount;
        int trailLen = 8;
        
        for (int t = 0; t < trailLen; t++) {
            int pos = (cometPos1 - t + stripCount) % stripCount;
            double fade = 1.0 - ((double) t / trailLen);
            fade = fade * fade; // quadratic falloff
            
            int r, g, b;
            if (t == 0) {
                // White-hot tip
                r = (int)(Math.min(allianceColor[0] + 180, 255) * masterBrightness);
                g = (int)(Math.min(allianceColor[1] + 180, 255) * masterBrightness);
                b = (int)(Math.min(allianceColor[2] + 180, 255) * masterBrightness);
            } else {
                r = (int)(allianceColor[0] * fade * masterBrightness);
                g = (int)(allianceColor[1] * fade * masterBrightness);
                b = (int)(allianceColor[2] * fade * masterBrightness);
            }
            
            int idx = stripStart + pos;
            ledBuffer[idx][0] = Math.max(ledBuffer[idx][0], r);
            ledBuffer[idx][1] = Math.max(ledBuffer[idx][1], g);
            ledBuffer[idx][2] = Math.max(ledBuffer[idx][2], b);
        }
        
        // Comet 2: reverse direction, offset by half the strip
        double speed2 = currentTime * 20.0;
        int cometPos2 = ((int)(stripCount / 2 - speed2) % stripCount + stripCount) % stripCount;
        
        for (int t = 0; t < trailLen; t++) {
            int pos = (cometPos2 + t + stripCount) % stripCount;
            double fade = 1.0 - ((double) t / trailLen);
            fade = fade * fade;
            
            int r, g, b;
            if (t == 0) {
                r = (int)(Math.min(allianceColor[0] + 180, 255) * masterBrightness);
                g = (int)(Math.min(allianceColor[1] + 180, 255) * masterBrightness);
                b = (int)(Math.min(allianceColor[2] + 180, 255) * masterBrightness);
            } else {
                r = (int)(allianceColor[0] * fade * masterBrightness);
                g = (int)(allianceColor[1] * fade * masterBrightness);
                b = (int)(allianceColor[2] * fade * masterBrightness);
            }
            
            int idx = stripStart + pos;
            ledBuffer[idx][0] = Math.max(ledBuffer[idx][0], r);
            ledBuffer[idx][1] = Math.max(ledBuffer[idx][1], g);
            ledBuffer[idx][2] = Math.max(ledBuffer[idx][2], b);
        }
        
        // Random sparkle accents -- 3 at any time, seeded by time
        long sparkSeed = (long)(currentTime * 10);
        for (int s = 0; s < 3; s++) {
            int sparkPos = (int)(((sparkSeed + s * 17) * 31 + s * 7) % stripCount);
            if (sparkPos < 0) sparkPos += stripCount;
            double sparkBright = (Math.sin(currentTime * 15 + s * 2.1) + 1) / 2;
            sparkBright = sparkBright * sparkBright * 0.5; // subtle
            int idx = stripStart + sparkPos;
            ledBuffer[idx][0] = Math.max(ledBuffer[idx][0], (int)(255 * sparkBright * masterBrightness));
            ledBuffer[idx][1] = Math.max(ledBuffer[idx][1], (int)(255 * sparkBright * masterBrightness));
            ledBuffer[idx][2] = Math.max(ledBuffer[idx][2], (int)(255 * sparkBright * masterBrightness));
        }
        
        pushBuffer();
    }
    
    /**
     * TELEOP MODE PATTERN - Smooth flowing gradient wave.
     * Two alliance-color bands flow slowly along the strip with smooth sinusoidal
     * shaping, creating a lava-lamp style "liquid flow" effect. Much calmer and
     * smoother than auto's racing comets -- clearly says "human in control."
     * Onboard LEDs breathe alliance color at a relaxed 4-second cycle.
     * Instantly says: "Driver has control"
     */
    private void setTeleopModePattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        clearBuffer();
        setOnboardBreathing(allianceColor);
        
        // Two flowing sine waves at different speeds create organic-looking movement
        double speed1 = currentTime * 0.4;  // slow drift
        double speed2 = currentTime * 0.25; // even slower counter-drift
        
        for (int i = 0; i < stripCount; i++) {
            double pos = (double) i / stripCount;
            
            // Two sine waves with different wavelengths and speeds
            double wave1 = (Math.sin((pos * 3.0 - speed1) * Math.PI * 2) + 1) / 2;
            double wave2 = (Math.sin((pos * 2.0 + speed2) * Math.PI * 2) + 1) / 2;
            
            // Combine with smooth max-like blending
            double combined = Math.max(wave1 * wave1, wave2 * wave2);
            
            // Map to brightness range 15% - 100% for smooth look
            double brightness = combined * 0.85 + 0.15;
            
            ledBuffer[stripStart + i][0] = (int)(allianceColor[0] * brightness * masterBrightness);
            ledBuffer[stripStart + i][1] = (int)(allianceColor[1] * brightness * masterBrightness);
            ledBuffer[stripStart + i][2] = (int)(allianceColor[2] * brightness * masterBrightness);
        }
        
        // Subtle white shimmer -- one bright point that slowly moves
        double shimmerPos = ((Math.sin(currentTime * 0.7) + 1) / 2) * stripCount;
        int shimmerIdx = (int) shimmerPos;
        double shimmerBright = (Math.sin(currentTime * 1.3) + 1) / 2 * 0.3;
        
        for (int s = -1; s <= 1; s++) {
            int idx = shimmerIdx + s;
            if (idx >= 0 && idx < stripCount) {
                double sFade = 1.0 - Math.abs(s) * 0.4;
                int pixel = stripStart + idx;
                ledBuffer[pixel][0] = Math.min(255, ledBuffer[pixel][0] + (int)(255 * shimmerBright * sFade * masterBrightness));
                ledBuffer[pixel][1] = Math.min(255, ledBuffer[pixel][1] + (int)(255 * shimmerBright * sFade * masterBrightness));
                ledBuffer[pixel][2] = Math.min(255, ledBuffer[pixel][2] + (int)(255 * shimmerBright * sFade * masterBrightness));
            }
        }
        
        pushBuffer();
    }
    
    /**
     * Updates LED pattern based on current state.
     * Sets currentPatternName for dashboard visibility.
     * checkEStop() is already called in periodic() -- not duplicated here.
     */
    private void updateLEDPattern() {
        double timeSinceStateStart = Timer.getFPGATimestamp() - stateStartTime;
        
        // the most important thing, always check for brownout first
        if (currentAction == ActionState.BROWNOUT) {
            currentPatternName = "Brownout";
            setBrownoutPattern();
            return;
        }
        // PRIORITY 0: First Active Alliance Flash!
        // When we first receive the game message, flash the first active alliance color
        // to alert drivers which alliance goes first. This is a quick 5-flash sequence.
        if (updateFirstAllianceFlash()) {
            currentPatternName = "First Alliance Flash";
            return;  // Currently flashing first alliance, don't do anything else
        }
        
        // PRIORITY 1: E-stop - ALWAYS highest priority (safety)
        // Note: checkEStop() already called in periodic(), just check the flag here
        if (isEStopped) {
            currentPatternName = "E-STOP";
            setEStopPattern();
            return;
        }
        
        // PRIORITY 2: Ready to shoot - Override everything (driver needs to know they can fire!)
        if (currentAction == ActionState.SHOOTING) {
            currentPatternName = "SHOOT NOW";
            setShootNowPattern();  // Solid green strobe - 
            return;
        }
        
        // Get game state info for time-based warnings
        double matchTime = DriverStation.getMatchTime();
        GamePhase phase = gameState.getGamePhase();
        boolean inEndgame = (matchTime > 0 && matchTime <= 30.0);
        
        // PRIORITY 3: Critical actions (aiming, spooling, climbing, intaking)
        // These override time warnings because robot is actively doing something important
        if (currentState == LEDState.AUTO || currentState == LEDState.TELEOP) {
            switch (currentAction) {
                case AIMING:
                    currentPatternName = "Targeting";
                    setTargetingPattern();
                    return;
                    
                case SPOOLING:
                    currentPatternName = "Spooling Up";
                    setPowerFillPattern();
                    return;
                    
                case CLIMBING:
                    currentPatternName = "Climbing";
                    setClimbRisingPattern();
                    return;
                    
                case INTAKING:
                    currentPatternName = "Intaking";
                    setIntakeFlowPattern();
                    return;
                    
                case DRIVING:
                case IDLE:
                    // Fall through to check time warnings
                    break;
                    
                case SHOOTING:
                case ESTOP:
                case BROWNOUT:
                    return;
                    
            }
        }
        
        // PRIORITY 4: Endgame warning (warnings BEFORE the 30-second endgame mark)
        // Only show if not doing critical actions
        // matchTime counts DOWN, so matchTime of 40 = 40 seconds left, 30 = endgame starts
        if (currentState == LEDState.TELEOP && !inEndgame && matchTime > 0) {
            // Time until endgame starts (positive means endgame hasn't started yet)
            double timeUntilEndgame = matchTime - 30.0;
            
            // 10 second warning before endgame (matchTime 37-40)
            if (timeUntilEndgame > 7.0 && timeUntilEndgame <= 10.0) {
                currentPatternName = "Endgame 10s Warning";
                setBreathing(Constants.LEDs.ENDGAME_WARNING_10SEC);
                return;
            }
            
            // 5 second warning before endgame (matchTime 33-35)
            if (timeUntilEndgame > 3.0 && timeUntilEndgame <= 5.0) {
                currentPatternName = "Endgame 5s Warning";
                setStrobe(Constants.LEDs.ENDGAME_WARNING_5SEC);
                return;
            }
            
            // 3 second warning before endgame (matchTime 30-33)
            if (timeUntilEndgame > 0.0 && timeUntilEndgame <= 3.0) {
                currentPatternName = "Endgame 3s Warning";
                setStrobe(Constants.LEDs.ENDGAME_WARNING_3SEC);
                return;
            }
        }
        
        // Once in endgame phase, show special endgame pattern (if idle)
        if (currentState == LEDState.TELEOP && phase == GamePhase.END_GAME && currentAction == ActionState.IDLE) {
            currentPatternName = "Endgame Urgency";
            setEndgameUrgencyPattern();
            return;
        }
        
        // PRIORITY 5: Alliance shift warnings (only during TELEOP and when idle/driving)
        // These use alliance-aware methods - only warn before OUR shifts
        if (currentState == LEDState.TELEOP) {
            // 5 second warning - "HEAD BACK" - start returning to scoring position
            // White breathing = get ready, our shift is coming!
            if (gameState.isHeadBackWarning()) {
                currentPatternName = "Head Back Warning";
                setBreathing(Constants.LEDs.SHIFT_WARNING_5SEC);
                return;
            }
            
            // 3 second warning - "GREEN LIGHT" - can start shooting!
            // Green strobe = you're cleared to shoot, shift is about to start!
            if (gameState.isGreenLightPreShift()) {
                currentPatternName = "Green Light Pre-Shift";
                setStrobe(Constants.LEDs.GREEN);
                return;
            }
            
            // Celebration when alliance just became active
            if (gameState.isOurAllianceActive()) {
                // First 2 seconds of active shift - rapid celebration
                double timeRemainingActive = gameState.getTimeRemainingActive();
                double timeSinceShiftStart = getShiftDuration(phase) - timeRemainingActive;
                if (timeSinceShiftStart >= 0 && timeSinceShiftStart <= 2.0) {
                    currentPatternName = "Shift Active Celebration";
                    setChase(Constants.LEDs.GREEN, allianceColor);
                    return;
                }
            }
        }
        
        // PRIORITY 6: Normal state patterns (pre-match sequence and basic operation)
        switch (currentState) {
            case BOOT_WARMUP:
                currentPatternName = "Boot Warmup";
                setTeamColorFlow();
                
                // After warmup duration, go straight to disabled
                if (timeSinceStateStart > Constants.LEDs.BOOT_ANIMATION_DURATION) {
                    setState(LEDState.DISABLED);
                }
                break;
                
            case NO_AUTO:
                currentPatternName = "No Auto Selected";
                setStrobe(Constants.LEDs.NO_AUTO_COLOR);
                break;
                
            case FMS_WAIT:
                currentPatternName = "Waiting for FMS";
                setStrobe(Constants.LEDs.FMS_DISCONNECTED_COLOR);
                
                // Re-check periodically - go to disabled instead of diagnostics
                if (timeSinceStateStart > 2.0) {
                    setState(LEDState.DISABLED);
                }
                break;
                
            case ALL_SYSTEMS_GO:
                currentPatternName = "All Systems Go";
                setReadySplitPattern();
                break;
                
            case DISABLED:
                currentPatternName = "Disabled (Waiting)";
                setWaitingPattern();
                break;
                
            case AUTO:
                currentPatternName = "Autonomous";
                setAutoModePattern();
                break;
                
            case TELEOP:
                currentPatternName = "Teleop";
                setTeleopModePattern();
                break;
                
            case ENDGAME:
                currentPatternName = "Endgame";
                setEndgameUrgencyPattern();
                break;
                
            case MATCH_END:
                currentPatternName = "Victory Celebration";
                setVictoryCelebration();
                
                // After celebration duration, go to normal disabled state
                if (timeSinceStateStart > matchEndCelebrationDuration) {
                    setState(LEDState.DISABLED);
                }
                break;
                
            case BROWNOUT:
                currentPatternName = "Brownout";
                setBrownoutPattern();
                break;
        }
    }
    
    @Override
    public void periodic() {
        // --- CANdle Boot Readiness Gate ---
        // The CANdle hardware needs ~1-2 seconds to initialize on the CAN bus after power-on.
        // Any commands sent before it's ready are silently lost. We wait for the minimum
        // boot time, then apply config on first ready detection.
        if (!candleReady) {
            double timeSinceBoot = Timer.getFPGATimestamp() - bootStartTime;
            if (timeSinceBoot < CANDLE_MIN_BOOT_SECONDS) {
                // Still in minimum boot window -- don't send anything
                DashboardHelper.putString(Category.DEBUG, "LED/Pattern", "Waiting for CANdle (" 
                    + String.format("%.1f", timeSinceBoot) + "s)");
                DashboardHelper.putString(Category.DEBUG, "LED/State", currentState.name());
                DashboardHelper.putString(Category.DEBUG, "LED/Action", currentAction.name());
                return;
            }
            // Minimum boot time elapsed -- apply config now
            candle.getConfigurator().apply(candleConfig);
            configApplied = true;
            candleReady = true;
            // Reset state start time so BOOT_WARMUP animation starts fresh from now
            stateStartTime = Timer.getFPGATimestamp();
        }
        
        // --- Dashboard Controls ---
        // Check for brightness updates from dashboard
        double newBrightness = DashboardHelper.getNumber(Category.DEBUG, "LED/Master_Brightness", masterBrightness);
        if (Math.abs(newBrightness - masterBrightness) > 0.01) {
            masterBrightness = Math.max(0.0, Math.min(1.0, newBrightness)); // Clamp 0-1
            DashboardHelper.putNumber(Category.DEBUG, "LED/Master_Brightness", masterBrightness);
        }
        
        // Check for test mode
        testMode = DashboardHelper.getBoolean(Category.DEBUG, "LED/Test_Mode", false);
        
        // Detect transition into test mode - clear all running animations to prevent conflicts
        if (testMode && !lastTestMode) {
            setSolidColor(Constants.LEDs.OFF);
        }
        lastTestMode = testMode;
        
        // If in test mode, skip normal LED operation
        if (testMode) {
            currentPatternName = "Test Mode";
            DashboardHelper.putString(Category.DEBUG, "LED/Pattern", currentPatternName);
            return;
        }
        
        // --- State Updates ---
        updateAllianceColor();
        checkEStop();
        
        // --- Pattern Dispatch ---
        updateLEDPattern();
        
        // --- Dashboard Telemetry ---
        DashboardHelper.putString(Category.DEBUG, "LED/Pattern", currentPatternName);
        DashboardHelper.putString(Category.DEBUG, "LED/State", currentState.name());
        DashboardHelper.putString(Category.DEBUG, "LED/Action", currentAction.name());
        
        // --- Critical Notifications ---
        double matchTime = DriverStation.getMatchTime();
        boolean inEndgame = (matchTime > 0 && matchTime <= 30.0);
        checkCriticalNotifications(matchTime, inEndgame);
    }
    
    /**
     * Sends critical notifications that drivers need to see immediately.
     * E-stop warnings, endgame alerts, etc.
     */
    private void checkCriticalNotifications(double matchTime, boolean inEndgame) {
        // E-STOP is the most critical - always notify!
        if (isEStopped && !hasNotifiedEStop) {
            hasNotifiedEStop = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.ERROR)
                .withTitle(" E-STOP ACTIVATED!")
                .withDescription("Robot is emergency stopped - check situation before resetting")
                .withDisplaySeconds(10.0));
        } else if (!isEStopped) {
            hasNotifiedEStop = false;
        }
        
        // Endgame warning at 30 seconds (only notify once per match)
        if (inEndgame && !hasNotifiedEndgame && matchTime <= 30.0 && matchTime > 28.0) {
            hasNotifiedEndgame = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle("ENDGAME!")
                .withDescription("30 seconds remaining - finish strong!")
                .withDisplaySeconds(3.0));
        }
        
        // Reset endgame notification when match resets
        if (matchTime > 30.0 || matchTime < 0) {
            hasNotifiedEndgame = false;
        }
    }
    
    // ==================== PUBLIC CONTROL METHODS ====================
    
    /**
     * Trigger match end celebration!
     * This sets the LED state to rapid team color strobe mode.
     * Call this when match ends (win or lose - celebrate your effort!)
     */
    public void triggerMatchEndCelebration() {
        setState(LEDState.MATCH_END);
    }
    
    /**
     * Set LEDs to solid team orange color.
     * Useful for pit display or special events.
     */
    public void setTeamOrange() {
        setSolidColor(Constants.LEDs.TEAM_SAFETY_ORANGE);
    }
    
    /**
     * Set LEDs to solid team blue color.
     * Useful for pit display or special events.
     */
    public void setTeamBlue() {
        setSolidColor(Constants.LEDs.TEAM_BLUE);
    }
    
    /**
     * Set LEDs to flowing team colors - orange and blue chase!
     * Great for pit display or idle states.
     */
    public void setTeamColorFlowMode() {
        setTeamColorFlow();
    }
}
