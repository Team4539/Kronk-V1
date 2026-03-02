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
 * LED subsystem using CTRE CANdle for visual feedback.
 * 
 * PHYSICAL LAYOUT (Y-spliced two-strip configuration):
 *   The CANdle data line is Y-spliced to two physical strips:
 *   - Strip A (top/back of robot): 38 LEDs
 *   - Strip B (belly pan): 58 LEDs (first 38 share IDs with Strip A, then 20 extra)
 *   - 8 onboard CANdle LEDs are also on the belly pan
 * 
 *   Indices 0-7:   Onboard LEDs (belly pan)
 *   Indices 8-45:  SHARED ZONE — visible on BOTH top and belly (38 LEDs)
 *   Indices 46-65: BELLY-ONLY ZONE — only on belly strip (20 LEDs)
 * 
 * VISUAL DESIGN PHILOSOPHY:
 *   Every animation is designed for maximum visual impact on WS2812B strips.
 *   Animations use smooth gradient blending, organic motion, and cinematic
 *   color transitions. Key techniques:
 *   - Color lerping for smooth gradients (no hard edges)
 *   - Quadratic/cubic fade curves for natural-looking trails
 *   - Layered sine waves for organic "living" motion
 *   - Additive blending for overlapping effects (fire, plasma, sparkle)
 *   - Highlight/accent colors on leading edges for visual pop
 * 
 * Provides status indicators for drivers and pit crew:
 * - Pre-match diagnostics (auto selected, FMS connection)
 * - Alliance colors during the match
 * - Action feedback (shooting, aiming, intaking, etc.)
 * - Warning flashes for shift timing and endgame
 * - Post-match celebration animation
 * 
 * Uses a state machine pattern: the main robot state (LEDState) controls
 * the base pattern, and temporary action overlays (ActionState) show
 * what the robot is actively doing.
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
    
    // CANdle boot readiness - hardware is not responsive until fully booted
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
    
    // CAN frame deduplication: only send when color actually changes
    private int lastSentR = -1, lastSentG = -1, lastSentB = -1;
    
    // LED update throttle: animations do not need 50Hz; 10Hz looks the same
    private int ledUpdateCounter = 0;
    private static final int LED_UPDATE_DIVISOR = 5; // Update every 5th cycle = 10Hz
    
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
        FIRING,         // Actively launching balls (feed motors running)
        SHOOTING,       // Ready to shoot (robot aimed, shooter spun up)
        AIMING,         // Aiming at target
        SPOOLING,       // Spinning up shooter wheels
        DRIVING,        // Just driving (lowest priority)
        INTAKING,       // Running intake
        CLIMBING,       // Running climber
        ESTOP,          // Emergency stop active
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
        candleConfig.LED.StripType = StripTypeValue.GRB;  // GRB is the standard WS2812B LED data format
        
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
     * Only sends a CAN frame if the color has actually changed to avoid bus spam.
     */
    private void setSolidColor(int[] color) {
        int r = (int)(color[0] * masterBrightness);
        int g = (int)(color[1] * masterBrightness);
        int b = (int)(color[2] * masterBrightness);
        // Skip if color hasn't changed to avoid flooding the CAN bus
        if (r == lastSentR && g == lastSentG && b == lastSentB) {
            return;
        }
        lastSentR = r;
        lastSentG = g;
        lastSentB = b;
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
    @SuppressWarnings("unused")
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
    @SuppressWarnings("unused")
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
    @SuppressWarnings("unused")
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
    // COLOR MATH UTILITIES
    // Smooth blending, gradients, and procedural noise
    // for cinematic LED effects.
    // ===========================================
    
    /**
     * Linearly interpolates between two RGB colors.
     * @param a First color {R, G, B}
     * @param b Second color {R, G, B}
     * @param t Blend factor (0.0 = fully a, 1.0 = fully b)
     * @return Blended color array {R, G, B}
     */
    private int[] lerpColor(int[] a, int[] b, double t) {
        t = Math.max(0.0, Math.min(1.0, t));
        return new int[] {
            (int)(a[0] + (b[0] - a[0]) * t),
            (int)(a[1] + (b[1] - a[1]) * t),
            (int)(a[2] + (b[2] - a[2]) * t)
        };
    }
    
    /**
     * Scales an RGB color by a brightness factor.
     * @param color Source color {R, G, B}
     * @param factor Brightness multiplier (0.0 - 1.0+)
     * @return Scaled color, clamped to 0-255
     */
    private int[] scaleColor(int[] color, double factor) {
        return new int[] {
            Math.min(255, Math.max(0, (int)(color[0] * factor))),
            Math.min(255, Math.max(0, (int)(color[1] * factor))),
            Math.min(255, Math.max(0, (int)(color[2] * factor)))
        };
    }
    
    /**
     * Additively blends a color into the LED buffer at the given index.
     * Existing color values are maxed (not replaced), creating a glow overlay effect.
     */
    private void blendAdditive(int index, int[] color, double intensity) {
        if (index < 0 || index >= ledCount) return;
        double br = intensity * masterBrightness;
        ledBuffer[index][0] = Math.min(255, Math.max(ledBuffer[index][0], (int)(color[0] * br)));
        ledBuffer[index][1] = Math.min(255, Math.max(ledBuffer[index][1], (int)(color[1] * br)));
        ledBuffer[index][2] = Math.min(255, Math.max(ledBuffer[index][2], (int)(color[2] * br)));
    }
    
    /**
     * 1D pseudo-noise function for organic-looking animation.
     * Returns a smooth value in [0, 1] that varies continuously with input.
     * Uses layered sine waves to approximate value noise.
     */
    private double noise1D(double x) {
        double v = Math.sin(x * 1.0) * 0.5
                 + Math.sin(x * 2.3 + 1.7) * 0.25
                 + Math.sin(x * 5.1 + 3.1) * 0.125;
        return (v / 0.875 + 1.0) / 2.0;  // Normalize to [0, 1]
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
    @SuppressWarnings("unused")
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
    @SuppressWarnings("unused")
    private void setAlternatingStrobe(int[] color1, int[] color2, double speed) {
        double currentTime = Timer.getFPGATimestamp();
        boolean showFirst = ((int)(currentTime / speed) % 2) == 0;
        
        setSolidColor(showFirst ? color1 : color2);
    }
    
    /**
     * Chase animation - flowing gradient segments of two colors.
     * Smooth sinusoidal blending between colors instead of hard-edged segments.
     * Creates a polished, flowing chase effect.
     */
    private void setChase(int[] color1, int[] color2) {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        double chaseOffset = currentTime * 1.5;
        
        clearBuffer();
        setOnboardColor(color1);
        
        for (int i = 0; i < stripCount; i++) {
            double pos = (double) i / stripCount;
            // Smooth sinusoidal blend between colors (6 segments worth)
            double blend = (Math.sin((pos * 6.0 - chaseOffset) * Math.PI) + 1.0) / 2.0;
            int[] pixelColor = lerpColor(color1, color2, blend);
            setLED(stripStart + i, scaleColor(pixelColor, masterBrightness));
        }
        
        pushBuffer();
    }
    
    /**
     * Team color flow - liquid metal gradient with twin comets.
     * A smooth continuous gradient of orange→gold→blue→deep blue flows along the
     * strip like molten metal. Two bright comet highlights (one orange, one blue)
     * travel in opposite directions, leaving warm/cool trails through the gradient.
     * More dynamic than the waiting aurora but still team-branded.
     */
    private void setTeamColorFlow() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        int[] orange = Constants.LEDs.TEAM_SAFETY_ORANGE;
        int[] gold = Constants.LEDs.TEAM_GOLD;
        int[] blue = Constants.LEDs.TEAM_BLUE;
        int[] deepBlue = Constants.LEDs.TEAM_BLUE_DEEP;
        
        clearBuffer();
        
        // Onboard: slow color morph orange→blue cycle
        double onboardPhase = (currentTime % 6.0) / 6.0;
        double onboardBlend = (Math.sin(onboardPhase * Math.PI * 2.0) + 1.0) / 2.0;
        int[] onboardColor = lerpColor(orange, blue, onboardBlend);
        setOnboardColor(onboardColor);
        
        // === Base gradient: smoothly flowing orange↔blue ===
        double flowSpeed = currentTime * 0.2;
        
        for (int i = 0; i < stripCount; i++) {
            double pos = (double) i / stripCount;
            
            // Create a 4-stop gradient: orange → gold → blue → deepBlue → orange (wrapping)
            double gradPos = (pos + flowSpeed) % 1.0;
            int[] baseColor;
            if (gradPos < 0.25) {
                baseColor = lerpColor(orange, gold, gradPos / 0.25);
            } else if (gradPos < 0.5) {
                baseColor = lerpColor(gold, blue, (gradPos - 0.25) / 0.25);
            } else if (gradPos < 0.75) {
                baseColor = lerpColor(blue, deepBlue, (gradPos - 0.5) / 0.25);
            } else {
                baseColor = lerpColor(deepBlue, orange, (gradPos - 0.75) / 0.25);
            }
            
            // Subtle brightness variation for depth
            double bright = 0.5 + 0.5 * noise1D(pos * 6.0 + currentTime * 0.8);
            setLED(stripStart + i, scaleColor(baseColor, bright * masterBrightness));
        }
        
        // === Orange highlight comet (forward) ===
        int trailLength = 14;
        double phaseA = (currentTime % 4.0) / 4.0;
        int headA = (int)(phaseA * stripCount) % stripCount;
        
        for (int t = 0; t < trailLength; t++) {
            int idx = ((headA - t) % stripCount + stripCount) % stripCount;
            double fade = 1.0 - ((double) t / trailLength);
            fade = fade * fade;
            int[] trailColor = (t == 0) ? Constants.LEDs.WARM_WHITE : scaleColor(orange, fade);
            blendAdditive(stripStart + idx, trailColor, (t == 0) ? 1.0 : fade);
        }
        
        // === Blue highlight comet (reverse) ===
        double phaseB = (1.0 - (currentTime % 4.0) / 4.0) % 1.0;
        int headB = (int)(phaseB * stripCount) % stripCount;
        
        for (int t = 0; t < trailLength; t++) {
            int idx = ((headB + t) % stripCount + stripCount) % stripCount;
            double fade = 1.0 - ((double) t / trailLength);
            fade = fade * fade;
            int[] trailColor = (t == 0) ? Constants.LEDs.COOL_WHITE : scaleColor(blue, fade);
            blendAdditive(stripStart + idx, trailColor, (t == 0) ? 1.0 : fade);
        }
        
        pushBuffer();
    }
    
    /**
     * Victory celebration - golden firework bursts with shimmer rain.
     * Periodic "explosions" of bright gold/white expand from random positions,
     * then cascade down as fading sparkle trails in team colors. Between bursts,
     * a gentle golden shimmer rain fills the strip. The overall effect is
     * celebratory and premium-feeling, like confetti and fireworks combined.
     */
    private void setVictoryCelebration() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        int[] gold = Constants.LEDs.VICTORY_GOLD;
        int[] orange = Constants.LEDs.TEAM_SAFETY_ORANGE;
        int[] blue = Constants.LEDs.TEAM_BLUE;
        int[] white = Constants.LEDs.WHITE;
        
        clearBuffer();
        
        // Onboard LEDs: rapid gold/white/blue cycle
        int onboardCycle = ((int)(currentTime * 8)) % 3;
        int[] onboardColor = (onboardCycle == 0) ? gold : (onboardCycle == 1) ? white : blue;
        setOnboardColor(onboardColor);
        
        // === Base layer: gentle golden shimmer rain ===
        for (int i = 0; i < stripCount; i++) {
            // Each LED twinkles independently using layered sine waves
            double twinkle = noise1D(i * 0.7 + currentTime * 2.5);
            twinkle = twinkle * twinkle;  // Sharper peaks = more sparkle-like
            
            // Cycle through gold/orange/blue based on position
            double colorCycle = noise1D(i * 0.3 + currentTime * 0.4);
            int[] baseColor;
            if (colorCycle > 0.6) {
                baseColor = gold;
            } else if (colorCycle > 0.3) {
                baseColor = orange;
            } else {
                baseColor = blue;
            }
            
            double brightness = twinkle * 0.4;  // Gentle base layer
            setLED(stripStart + i, scaleColor(baseColor, brightness * masterBrightness));
        }
        
        // === Burst layer: periodic firework explosions ===
        // 3 concurrent bursts with staggered timing
        for (int burst = 0; burst < 3; burst++) {
            double burstCycle = 1.8;  // Each burst repeats every 1.8 seconds
            double burstTime = (currentTime + burst * 0.6) % burstCycle;
            
            // Burst center: pseudo-random position seeded by burst cycle count
            long burstSeed = (long)((currentTime + burst * 0.6) / burstCycle);
            int burstCenter = (int)(((burstSeed * 31 + burst * 17) % stripCount + stripCount) % stripCount);
            
            if (burstTime < 0.8) {
                // Active burst: expanding ring of light
                double burstProgress = burstTime / 0.8;
                int burstRadius = (int)(burstProgress * stripCount / 3);
                double burstBright = 1.0 - burstProgress;  // Fades as it expands
                burstBright = burstBright * burstBright;
                
                // Pick burst color based on seed
                int[] burstColor = ((burstSeed + burst) % 3 == 0) ? gold 
                                 : ((burstSeed + burst) % 3 == 1) ? orange : white;
                
                for (int r = -burstRadius; r <= burstRadius; r++) {
                    int idx = ((burstCenter + r) % stripCount + stripCount) % stripCount;
                    double distFade = 1.0 - (double)Math.abs(r) / (burstRadius + 1);
                    distFade = distFade * distFade;
                    double pixelBright = burstBright * distFade;
                    
                    // Leading edge is white-hot
                    int[] pixelColor = (Math.abs(r) >= burstRadius - 1) 
                                     ? lerpColor(burstColor, white, 0.7) : burstColor;
                    blendAdditive(stripStart + idx, pixelColor, pixelBright);
                }
            }
        }
        
        pushBuffer();
    }
    
    // ===========================================
    // CREATIVE INFORMATIVE PATTERNS
    // Designed for instant recognizability and visual impact.
    // ===========================================
    
    /**
     * E-STOP PATTERN - Aggressive plasma fire with unstable flickering.
     * The strip looks like it's on fire — hot red/orange plasma roils
     * across the LEDs with random intensity spikes and dark patches.
     * A fast strobe overlay makes it impossible to ignore.
     * Onboard LEDs strobe bright red for maximum alarm visibility.
     */
    private void setEStopPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        clearBuffer();
        
        // Onboard LEDs: aggressive red strobe
        boolean onboardOn = ((int)(currentTime * 10) % 2) == 0;
        setOnboardColor(onboardOn ? Constants.LEDs.ESTOP_COLOR : Constants.LEDs.OFF);
        
        // Global strobe overlay: whole strip flashes dark periodically
        double strobePhase = (currentTime * 4.0) % 1.0;
        double strobeMod = (strobePhase < 0.15) ? 0.15 : 1.0;  // Brief dark flash
        
        // Plasma fire effect: each LED has independent turbulent brightness
        for (int i = 0; i < stripCount; i++) {
            // Multiple layered noise sources for turbulent fire
            double fire1 = Math.sin(currentTime * 8.0 + i * 0.8) * 0.5 + 0.5;
            double fire2 = Math.sin(currentTime * 13.0 + i * 1.3 + 2.0) * 0.5 + 0.5;
            double fire3 = Math.sin(currentTime * 21.0 + i * 0.4 + 5.0) * 0.5 + 0.5;
            
            // Combine for chaotic fire brightness
            double fireBright = fire1 * 0.4 + fire2 * 0.35 + fire3 * 0.25;
            fireBright = fireBright * fireBright;  // Square for more contrast
            
            // Occasional bright flare-ups (unstable power feel)
            double flare = Math.sin(currentTime * 31.0 + i * 3.7);
            if (flare > 0.92) {
                fireBright = 1.0;  // Full bright spike!
            }
            
            fireBright = Math.max(0.08, fireBright) * strobeMod;
            
            // Color: blend between deep red, hot orange-red, and white-hot
            int[] color;
            if (fireBright > 0.8) {
                // White-hot core
                color = lerpColor(Constants.LEDs.ESTOP_CORE, Constants.LEDs.WARM_WHITE, 
                                  (fireBright - 0.8) / 0.2);
            } else if (fireBright > 0.4) {
                // Orange-red flame
                color = lerpColor(Constants.LEDs.ESTOP_COLOR, Constants.LEDs.ESTOP_CORE, 
                                  (fireBright - 0.4) / 0.4);
            } else {
                // Deep red embers
                color = scaleColor(Constants.LEDs.ESTOP_COLOR, fireBright / 0.4 * 0.6 + 0.1);
            }
            
            setLED(stripStart + i, scaleColor(color, masterBrightness));
        }
        
        pushBuffer();
    }
    
    /**
     * TARGETING PATTERN - Radar sweep with tightening lock-on glow.
     * A bright amber beam sweeps back and forth across the strip, leaving a
     * fading phosphor trail like a radar display. As it sweeps repeatedly,
     * a warm glow builds at the center indicating "locking on". When the
     * brackets meet, the strip flashes bright to show LOCKED.
     */
    private void setTargetingPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        // Update sweep position at 20Hz
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
        
        // Onboard LEDs: pulsing amber
        setOnboardBreathing(Constants.LEDs.AIMING_COLOR);
        
        int[] amber = Constants.LEDs.AIMING_COLOR;
        int[] highlight = Constants.LEDs.AIMING_HIGHLIGHT;
        
        // When brackets meet in center, flash bright to show "LOCKED"
        if (targetingPosition >= center - 2) {
            // Bright lock flash — all amber with white-hot center
            for (int i = 0; i < stripCount; i++) {
                double distFromCenter = Math.abs(i - center) / (double) center;
                int[] lockColor = lerpColor(highlight, amber, distFromCenter);
                setLED(stripStart + i, scaleColor(lockColor, masterBrightness));
            }
        } else {
            // === Dim base glow (radar screen phosphor) ===
            for (int i = 0; i < stripCount; i++) {
                setLED(stripStart + i, scaleColor(amber, 0.04 * masterBrightness));
            }
            
            // === Center crosshair glow — builds intensity as brackets close in ===
            double lockProgress = (double) targetingPosition / center;  // 0→1 as brackets close
            double centerGlow = lockProgress * lockProgress * 0.6;  // Quadratic build
            double centerPulse = (Math.sin(currentTime * Math.PI * 5) + 1) / 2;
            centerGlow *= (0.7 + 0.3 * centerPulse);
            
            for (int c = -3; c <= 3; c++) {
                int idx = center + c;
                if (idx >= 0 && idx < stripCount) {
                    double falloff = 1.0 - Math.abs(c) / 4.0;
                    blendAdditive(stripStart + idx, amber, centerGlow * falloff);
                }
            }
            
            // === Left sweep beam: bright head + phosphor trail ===
            int sweepWidth = 3;
            int trailLen = Math.max(4, targetingPosition / 2);
            
            for (int t = 0; t < trailLen + sweepWidth; t++) {
                int pos = targetingPosition - t;
                if (pos >= 0 && pos < stripCount) {
                    double fade;
                    if (t < sweepWidth) {
                        // Bright sweep head
                        fade = 1.0;
                        blendAdditive(stripStart + pos, highlight, fade * masterBrightness);
                    } else {
                        // Phosphor trail: slow exponential decay
                        fade = Math.pow(0.82, t - sweepWidth);
                        blendAdditive(stripStart + pos, amber, fade * 0.7 * masterBrightness);
                    }
                }
            }
            
            // === Right sweep beam (mirror) ===
            int rightPos = stripCount - 1 - targetingPosition;
            for (int t = 0; t < trailLen + sweepWidth; t++) {
                int pos = rightPos + t;
                if (pos >= 0 && pos < stripCount) {
                    double fade;
                    if (t < sweepWidth) {
                        fade = 1.0;
                        blendAdditive(stripStart + pos, highlight, fade * masterBrightness);
                    } else {
                        fade = Math.pow(0.82, t - sweepWidth);
                        blendAdditive(stripStart + pos, amber, fade * 0.7 * masterBrightness);
                    }
                }
            }
        }
        pushBuffer();
    }
    
    /**
     * POWER FILL PATTERN - Plasma energy charging from edges to center.
     * Energy streams flow inward from both strip edges, building intensity
     * as they converge. The filled portion shimmers with electric crackling
     * effects. A bright white-hot leading edge shows the charge front.
     * Onboard LEDs pulse faster as charge builds, creating visual tension.
     */
    private void setPowerFillPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;
        
        // Increment fill tracking at 10Hz
        if (currentTime - lastFillUpdate > 0.1) {
            lastFillUpdate = currentTime;
            fillLevel++;
            if (fillLevel > center) {
                fillLevel = 0;
            }
        }
        
        clearBuffer();
        
        int[] chargeColor = Constants.LEDs.SPOOLING_COLOR;
        int[] highlight = Constants.LEDs.SPOOLING_HIGHLIGHT;
        
        // Onboard LEDs: pulse speed increases with fill level
        double fillRatio = (double) fillLevel / center;
        double pulseSpeed = 2.0 + fillRatio * 8.0;
        double onboardPulse = (Math.sin(currentTime * Math.PI * pulseSpeed) + 1) / 2;
        double onboardBright = (0.2 + 0.8 * onboardPulse) * masterBrightness;
        int onboard = Constants.LEDs.ONBOARD_LED_COUNT;
        for (int i = 0; i < onboard && i < ledCount; i++) {
            ledBuffer[i][0] = (int)(chargeColor[0] * onboardBright);
            ledBuffer[i][1] = (int)(chargeColor[1] * onboardBright);
            ledBuffer[i][2] = (int)(chargeColor[2] * onboardBright);
        }
        
        // === Charging from both edges toward center ===
        for (int i = 0; i < stripCount; i++) {
            int distFromNearestEdge = Math.min(i, stripCount - 1 - i);
            
            // "Filled" = within fillLevel distance from either edge
            boolean isFilled = distFromNearestEdge < fillLevel;
            
            if (isFilled) {
                // Filled portion: shimmering energy with electric crackle
                double shimmer = 0.6 + 0.4 * noise1D(i * 0.5 + currentTime * 8.0);
                
                // Random electric crackle: brief bright flashes
                double crackle = Math.sin(currentTime * 25.0 + i * 3.7);
                if (crackle > 0.93) {
                    shimmer = 1.0;
                }
                
                // Intensity builds toward center (inner filled areas are brighter)
                double depthFade = 0.5 + 0.5 * ((double)distFromNearestEdge / (fillLevel + 1));
                
                int[] pixelColor = lerpColor(chargeColor, highlight, shimmer * 0.3);
                setLED(stripStart + i, scaleColor(pixelColor, shimmer * depthFade * masterBrightness));
                
                // Leading edge (at fillLevel distance from edge): white-hot
                if (distFromNearestEdge >= fillLevel - 2 && distFromNearestEdge <= fillLevel) {
                    double edgeBright = 1.0 - (double)(fillLevel - distFromNearestEdge) * 0.3;
                    blendAdditive(stripStart + i, highlight, edgeBright * masterBrightness);
                }
            } else {
                // Unfilled: very dim glow that hints at what's coming
                double dimGlow = 0.03 + 0.02 * noise1D(i * 0.3 + currentTime * 1.0);
                setLED(stripStart + i, scaleColor(chargeColor, dimGlow * masterBrightness));
            }
        }
        
        // Center convergence: bright pulse when nearly full
        if (fillRatio > 0.8) {
            double convergeBright = (fillRatio - 0.8) / 0.2;
            convergeBright *= (Math.sin(currentTime * Math.PI * 12) + 1) / 2;
            for (int c = -2; c <= 2; c++) {
                int idx = center + c;
                if (idx >= 0 && idx < stripCount) {
                    blendAdditive(stripStart + idx, Constants.LEDs.WARM_WHITE, 
                                  convergeBright * (1.0 - Math.abs(c) * 0.3) * masterBrightness);
                }
            }
        }
        
        pushBuffer();
    }
    
    /**
     * INTAKE FLOW PATTERN - Smooth particle streams converging to center.
     * Multiple soft pink-magenta particles flow from both edges toward center,
     * with gentle speed variation that makes them look organic (not mechanical).
     * When particles reach center they produce a brief bright convergence flash.
     * The effect looks like energy being sucked inward — perfect for intake.
     */
    private void setIntakeFlowPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;
        
        int[] color = Constants.LEDs.INTAKING_COLOR;
        int[] highlight = Constants.LEDs.INTAKING_HIGHLIGHT;
        
        clearBuffer();
        setOnboardBreathing(color);
        
        // Dim base glow so strip isn't dead black
        for (int i = 0; i < stripCount; i++) {
            setLED(stripStart + i, scaleColor(color, 0.04 * masterBrightness));
        }
        
        // === Multiple particle streams from each side ===
        int numParticles = 4;  // Per side
        double cycleDuration = 0.8;  // Seconds per full transit
        
        for (int p = 0; p < numParticles; p++) {
            double offset = (double) p / numParticles;
            double particlePhase = ((currentTime / cycleDuration) + offset) % 1.0;
            
            // Organic speed: particles accelerate as they approach center
            double eased = particlePhase * particlePhase;  // Ease-in (starts slow, speeds up)
            
            int leftPos = (int)(eased * center);
            int rightPos = stripCount - 1 - leftPos;
            
            int trailLen = 6;
            
            // Left particle flowing right toward center
            for (int t = 0; t < trailLen; t++) {
                int idx = leftPos - t;
                if (idx >= 0 && idx < center) {
                    double fade = 1.0 - ((double) t / trailLen);
                    fade = fade * fade;
                    int[] trailColor = (t == 0) ? highlight : color;
                    blendAdditive(stripStart + idx, trailColor, fade * 0.9);
                }
            }
            
            // Right particle flowing left toward center
            for (int t = 0; t < trailLen; t++) {
                int idx = rightPos + t;
                if (idx >= center && idx < stripCount) {
                    double fade = 1.0 - ((double) t / trailLen);
                    fade = fade * fade;
                    int[] trailColor = (t == 0) ? highlight : color;
                    blendAdditive(stripStart + idx, trailColor, fade * 0.9);
                }
            }
        }
        
        // === Center convergence glow ===
        // Continuous soft pulse at center from all the converging particles
        double convergePulse = 0.0;
        for (int p = 0; p < numParticles; p++) {
            double offset = (double) p / numParticles;
            double particlePhase = ((currentTime / cycleDuration) + offset) % 1.0;
            // Bright flash when particle reaches center (phase near 1.0)
            if (particlePhase > 0.85) {
                double flashIntensity = (particlePhase - 0.85) / 0.15;
                flashIntensity = flashIntensity * flashIntensity;
                convergePulse = Math.max(convergePulse, flashIntensity);
            }
        }
        
        for (int c = -3; c <= 3; c++) {
            int idx = center + c;
            if (idx >= 0 && idx < stripCount) {
                double spread = 1.0 - Math.abs(c) / 4.0;
                blendAdditive(stripStart + idx, highlight, convergePulse * spread * 0.8);
            }
        }
        
        pushBuffer();
    }
    
    /**
     * CLIMB RISING PATTERN - Purple fire columns rising upward.
     * A fire-like effect where violet flames lick upward along the strip.
     * Multiple flame "tongues" at different heights with organic flickering
     * and lavender-white tips. The base is deep purple, tips are bright lavender.
     * Conveys upward motion and effort — perfect for climbing.
     */
    private void setClimbRisingPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        int[] base = Constants.LEDs.CLIMBING_COLOR;
        int[] tip = Constants.LEDs.CLIMBING_HIGHLIGHT;
        
        clearBuffer();
        setOnboardBreathing(base);
        
        // === Fire simulation: each LED's brightness based on "flame height" ===
        for (int i = 0; i < stripCount; i++) {
            double pos = (double) i / stripCount;
            
            // Multiple flame tongues at different speeds (rising upward = increasing index)
            double flame1 = Math.sin((pos * 3.0 - currentTime * 1.5) * Math.PI * 2.0);
            double flame2 = Math.sin((pos * 5.0 - currentTime * 2.0 + 1.0) * Math.PI * 2.0);
            double flame3 = Math.sin((pos * 2.0 - currentTime * 1.0 + 3.0) * Math.PI * 2.0);
            
            // Combine and shape: positive values = flame visible
            double flameIntensity = flame1 * 0.4 + flame2 * 0.35 + flame3 * 0.25;
            flameIntensity = (flameIntensity + 1.0) / 2.0;  // [0, 1]
            
            // Gradient: base is always somewhat lit, flames add brightness
            double baseBright = 0.15 + 0.15 * noise1D(i * 0.4 + currentTime * 3.0);
            double totalBright = Math.min(1.0, baseBright + flameIntensity * 0.7);
            
            // Color: blend from deep purple (dim) to lavender-white (bright flames)
            int[] pixelColor = lerpColor(base, tip, totalBright);
            setLED(stripStart + i, scaleColor(pixelColor, totalBright * masterBrightness));
        }
        
        // === Bright rising comets (3 evenly spaced) for extra upward motion ===
        int numComets = 3;
        double risePhase = (currentTime % 2.0) / 2.0;
        int trailLen = 6;
        
        for (int c = 0; c < numComets; c++) {
            double cometPhase = (risePhase + (double) c / numComets) % 1.0;
            int risePos = (int)(cometPhase * stripCount);
            
            for (int t = 0; t < trailLen; t++) {
                int pos = risePos - t;
                if (pos >= 0 && pos < stripCount) {
                    double fade = 1.0 - ((double) t / trailLen);
                    fade = fade * fade;
                    int[] cometColor = (t == 0) ? tip : base;
                    blendAdditive(stripStart + pos, cometColor, fade * 0.7);
                }
            }
        }
        
        pushBuffer();
    }
    
    /**
     * READY SPLIT PATTERN - Team identity display with animated boundary.
     * Left half glows warm orange, right half glows cool blue, with a bright
     * white energy line at the boundary that breathes and sparkles. The two
     * halves have subtle inner shimmer for depth. Green onboard LEDs = READY.
     */
    private void setReadySplitPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;
        
        clearBuffer();
        setOnboardColor(Constants.LEDs.GREEN);  // Green = READY
        
        int[] orange = Constants.LEDs.TEAM_SAFETY_ORANGE;
        int[] blue = Constants.LEDs.TEAM_BLUE;
        
        for (int i = 0; i < stripCount; i++) {
            int[] baseColor = (i < center) ? orange : blue;
            
            // Subtle inner shimmer for depth
            double shimmer = 0.7 + 0.3 * noise1D(i * 0.5 + currentTime * 1.5);
            setLED(stripStart + i, scaleColor(baseColor, shimmer * masterBrightness));
        }
        
        // Animated boundary: breathing white energy line with sparkle
        double breathPhase = (currentTime % 2.0) / 2.0;
        double breathBright = 0.5 + 0.5 * ((Math.sin(breathPhase * Math.PI * 2) + 1) / 2);
        
        // Boundary glow: 5 LEDs wide, fading from center
        for (int b = -2; b <= 2; b++) {
            int idx = center + b;
            if (idx >= 0 && idx < stripCount) {
                double falloff = 1.0 - Math.abs(b) / 3.0;
                double bright = breathBright * falloff;
                blendAdditive(stripStart + idx, Constants.LEDs.WARM_WHITE, bright * masterBrightness);
            }
        }
        
        // Occasional sparkle on the boundary line
        double sparkle = Math.sin(currentTime * 13.0);
        if (sparkle > 0.85) {
            int sparkIdx = center + (int)(Math.sin(currentTime * 7.0) * 2);
            if (sparkIdx >= 0 && sparkIdx < stripCount) {
                blendAdditive(stripStart + sparkIdx, Constants.LEDs.WHITE, masterBrightness);
            }
        }
        
        pushBuffer();
    }
    
    /**
     * ENDGAME URGENCY PATTERN - Accelerating heartbeat with panic strobe.
     * Alliance-colored pulse that starts as a slow heartbeat (lub-dub) and
     * accelerates as match time decreases. Periodic white burst flashes
     * interrupt the heartbeat for extra urgency. The effect builds panic
     * progressively — drivers can feel the time pressure.
     */
    private void setEndgameUrgencyPattern() {
        double currentTime = Timer.getFPGATimestamp();
        double matchTime = DriverStation.getMatchTime();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        // Every 3 seconds, do a quick white flash burst
        if (currentTime - lastEndgameFlash > 3.0) {
            lastEndgameFlash = currentTime;
            endgameFlashCount = 3;
        }
        
        clearBuffer();
        
        // Onboard: fast alliance strobe
        boolean onboardOn = ((int)(currentTime * 8) % 2) == 0;
        setOnboardColor(onboardOn ? allianceColor : Constants.LEDs.WHITE);
        
        if (endgameFlashCount > 0) {
            endgameFlashCount--;
            // White burst flash across entire strip
            double flashBright = 0.7 + 0.3 * ((endgameFlashCount % 2 == 0) ? 1.0 : 0.0);
            for (int i = 0; i < stripCount; i++) {
                setLED(stripStart + i, scaleColor(Constants.LEDs.WHITE, flashBright * masterBrightness));
            }
        } else {
            // Heartbeat speed scales with remaining time (1.2Hz to 5Hz)
            double urgency = 1.0 - Math.max(0, Math.min(1.0, matchTime / 30.0));
            double heartRate = 1.2 + urgency * 3.8;
            double beatPhase = (currentTime * heartRate) % 1.0;
            
            // Double-beat heartbeat (lub-dub)
            double beatIntensity;
            if (beatPhase < 0.08) {
                beatIntensity = beatPhase / 0.08;
            } else if (beatPhase < 0.16) {
                beatIntensity = 1.0 - (beatPhase - 0.08) / 0.08;
            } else if (beatPhase < 0.24) {
                beatIntensity = (beatPhase - 0.16) / 0.08 * 0.6;
            } else if (beatPhase < 0.32) {
                beatIntensity = 0.6 * (1.0 - (beatPhase - 0.24) / 0.08);
            } else {
                beatIntensity = 0;
            }
            
            // Apply heartbeat with center-bright radial gradient
            for (int i = 0; i < stripCount; i++) {
                double distFromCenter = Math.abs(i - stripCount / 2) / (double)(stripCount / 2);
                double radial = 1.0 - distFromCenter * 0.4;
                double brightness = Math.max(0.04, beatIntensity * radial);
                
                // As urgency builds, add a frantic flicker overlay
                if (urgency > 0.6) {
                    double flicker = noise1D(i * 0.8 + currentTime * 15.0);
                    brightness += (urgency - 0.6) / 0.4 * flicker * 0.3;
                }
                
                brightness = Math.min(1.0, brightness);
                
                // Blend toward white at high intensities for drama
                int[] color = (beatIntensity > 0.7) 
                    ? lerpColor(allianceColor, Constants.LEDs.WHITE, (beatIntensity - 0.7) / 0.3 * 0.4)
                    : allianceColor;
                setLED(stripStart + i, scaleColor(color, brightness * masterBrightness));
            }
        }
        pushBuffer();
    }
    
    /**
     * BROWNOUT PATTERN - Cinematic dying power with amber sparks.
     * Sections of the strip dim out in slow waves, as if power is failing.
     * Occasional bright amber "sparks" fight to stay alive, creating a
     * dramatic sputtering effect. The overall brightness slowly oscillates
     * to simulate unstable voltage. Feels like the robot is losing power.
     */
    private void setBrownoutPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        int[] brown = Constants.LEDs.BROWNOUT_COLOR;
        int[] spark = Constants.LEDs.BROWNOUT_SPARK;
        
        clearBuffer();
        
        // Onboard LEDs: dim flickering brown
        double onboardFlicker = noise1D(currentTime * 6.0) * 0.5 + 0.1;
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, scaleColor(brown, onboardFlicker * masterBrightness));
        }
        
        // Global voltage instability: slow oscillation of base brightness
        double voltageWave = 0.3 + 0.4 * ((Math.sin(currentTime * 0.8) + 1) / 2);
        
        for (int i = 0; i < stripCount; i++) {
            double pos = (double) i / stripCount;
            
            // Blackout zone: sections go dark based on overlapping waves
            double blackout1 = Math.sin((pos * 3.0 - currentTime * 0.4) * Math.PI * 2.0);
            double blackout2 = Math.sin((pos * 5.0 + currentTime * 0.7) * Math.PI * 2.0);
            double blackout = (blackout1 + blackout2) / 2.0;
            
            // Map to brightness: negative = dark, positive = lit
            double sectionBright;
            if (blackout < -0.2) {
                sectionBright = 0.02;  // Nearly dead
            } else if (blackout < 0.2) {
                // Sputtering zone: flickering between dim and medium
                double sputter = noise1D(i * 0.7 + currentTime * 8.0);
                sectionBright = 0.05 + sputter * 0.3;
            } else {
                sectionBright = 0.3 + blackout * 0.4;
            }
            
            sectionBright *= voltageWave;
            
            // Occasional bright amber spark
            double sparkChance = Math.sin(currentTime * 23.0 + i * 4.1);
            if (sparkChance > 0.94) {
                // Bright spark!
                double sparkBright = (sparkChance - 0.94) / 0.06;
                setLED(stripStart + i, scaleColor(spark, sparkBright * masterBrightness));
            } else {
                setLED(stripStart + i, scaleColor(brown, sectionBright * masterBrightness));
            }
        }
        
        pushBuffer();
    }
    
    /**
     * WAITING/DISABLED PATTERN - Smooth aurora flow with team colors.
     * 
     * A mesmerizing "breathing aurora" of orange and blue that flows organically
     * across the strip. Multiple layered sine waves create a liquid, living
     * gradient that slowly drifts and morphs. Each LED smoothly blends between
     * team orange and team blue based on its position and time, with a gentle
     * overall brightness wave that gives the effect a breathing quality.
     * 
     * Onboard LEDs (0-7): Soft alternating orange/blue breathing glow.
     * Strip LEDs (8-65): Smooth flowing aurora gradient across entire strip.
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
        double breathPhase = (currentTime % 4.0) / 4.0;
        double breathBrightness = 0.3 + 0.7 * ((Math.sin(breathPhase * Math.PI * 2.0) + 1.0) / 2.0);
        
        for (int i = 0; i < onboard && i < ledCount; i++) {
            int[] color = (i < onboard / 2) ? orange : blue;
            ledBuffer[i][0] = (int)(color[0] * breathBrightness * masterBrightness);
            ledBuffer[i][1] = (int)(color[1] * breathBrightness * masterBrightness);
            ledBuffer[i][2] = (int)(color[2] * breathBrightness * masterBrightness);
        }
        
        // === STRIP LEDs: Flowing aurora gradient ===
        // Multiple sine waves at different frequencies create organic motion.
        // Each LED blends between orange and blue based on a composite wave.
        double drift1 = currentTime * 0.15;   // Very slow primary drift
        double drift2 = currentTime * 0.08;   // Even slower counter-drift
        double drift3 = currentTime * 0.25;   // Subtle faster ripple
        
        // Global breathing envelope (slow brightness wave across entire strip)
        double globalBreath = 0.6 + 0.4 * ((Math.sin(currentTime * 0.5) + 1.0) / 2.0);
        
        for (int i = 0; i < stripCount; i++) {
            double pos = (double) i / stripCount;
            
            // Layered sine waves for organic color blending
            double wave1 = Math.sin((pos * 2.5 - drift1) * Math.PI * 2.0);
            double wave2 = Math.sin((pos * 1.5 + drift2) * Math.PI * 2.0);
            double wave3 = Math.sin((pos * 4.0 - drift3) * Math.PI * 2.0) * 0.3;
            
            // Combine into a smooth blend factor [0, 1] where 0=orange, 1=blue
            double blend = (wave1 * 0.5 + wave2 * 0.35 + wave3 * 0.15 + 1.0) / 2.0;
            blend = Math.max(0.0, Math.min(1.0, blend));
            
            // Smooth the blend curve for more pleasing transitions
            blend = blend * blend * (3.0 - 2.0 * blend);  // Smoothstep
            
            // Lerp between orange and blue
            int[] color = lerpColor(orange, blue, blend);
            
            // Per-LED brightness variation for depth
            double localBright = noise1D(pos * 8.0 + currentTime * 0.6);
            localBright = 0.4 + 0.6 * localBright;  // Range: 0.4 - 1.0
            
            double finalBright = globalBreath * localBright * masterBrightness;
            ledBuffer[stripStart + i][0] = (int)(color[0] * finalBright);
            ledBuffer[stripStart + i][1] = (int)(color[1] * finalBright);
            ledBuffer[stripStart + i][2] = (int)(color[2] * finalBright);
        }
        
        pushBuffer();
    }
    
    /**
     * SHOOT NOW PATTERN - Pulsing green energy radiating from center.
     * The strip pulses with vivid green energy that radiates outward from center
     * in smooth waves. A bright mint-white core pulses rapidly at center, with
     * green waves flowing outward and fading. The effect screams "FIRE NOW" —
     * energized and ready. Multiple overlapping rings create a continuous flow.
     */
    private void setShootNowPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;
        
        int[] color = Constants.LEDs.SHOOTING_COLOR;
        int[] highlight = Constants.LEDs.SHOOTING_HIGHLIGHT;
        
        clearBuffer();
        setOnboardColor(color);
        
        // Dim green base
        for (int i = 0; i < stripCount; i++) {
            setLED(stripStart + i, scaleColor(color, 0.06 * masterBrightness));
        }
        
        // === Outward-radiating rings ===
        int numRings = 3;
        for (int ring = 0; ring < numRings; ring++) {
            double ringPhase = (currentTime * 2.5 + ring * (1.0 / numRings)) % 1.0;
            int ringPos = (int)(ringPhase * center);
            int trailLen = 8;
            
            for (int t = 0; t < trailLen; t++) {
                int pos = ringPos - t;
                if (pos >= 0 && pos < center) {
                    double fade = 1.0 - ((double) t / trailLen);
                    fade = fade * fade;
                    
                    // Leading edge is bright mint-white
                    int[] ringColor = (t < 2) ? lerpColor(color, highlight, 0.6 - t * 0.3) : color;
                    
                    // Left of center
                    blendAdditive(stripStart + center - pos, ringColor, fade * 0.9);
                    // Right of center
                    blendAdditive(stripStart + center + pos, ringColor, fade * 0.9);
                }
            }
        }
        
        // === Bright pulsing center core ===
        double corePulse = (Math.sin(currentTime * Math.PI * 6) + 1) / 2 * 0.4 + 0.6;
        for (int c = -2; c <= 2; c++) {
            int idx = center + c;
            if (idx >= 0 && idx < stripCount) {
                double falloff = 1.0 - Math.abs(c) / 3.0;
                blendAdditive(stripStart + idx, highlight, corePulse * falloff * masterBrightness);
            }
        }
        
        pushBuffer();
    }
    
    /**
     * FIRING PATTERN - Explosive energy blast with golden lightning.
     * Rapid muzzle flash strobes alternate with golden lightning bolts that
     * propagate outward from center. Between flashes, fast green-to-gold
     * shockwaves cascade across the strip. Maximum visual intensity — the
     * robot is actively scoring and everyone should see it.
     */
    private void setFiringPattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;
        
        int[] green = Constants.LEDs.SHOOTING_COLOR;
        int[] gold = Constants.LEDs.TEAM_GOLD;
        
        clearBuffer();
        
        // Onboard LEDs: rapid white/green strobe (muzzle flash)
        boolean onboardFlash = ((int)(currentTime * 20) % 2) == 0;
        setOnboardColor(onboardFlash ? Constants.LEDs.WHITE : green);
        
        // Muzzle flash cycle: 0.12s per flash
        double flashPhase = (currentTime % 0.12) / 0.12;
        boolean inFlash = flashPhase < 0.25;
        
        if (inFlash) {
            // FLASH — bright white burst fading to green
            double flashIntensity = 1.0 - (flashPhase / 0.25);
            flashIntensity = flashIntensity * flashIntensity;
            
            for (int i = 0; i < stripCount; i++) {
                int[] flashColor = lerpColor(green, Constants.LEDs.WHITE, flashIntensity * 0.7);
                setLED(stripStart + i, scaleColor(flashColor, flashIntensity * masterBrightness));
            }
        } else {
            // Between flashes: fast golden lightning bolts outward from center
            int numBolts = 4;
            for (int bolt = 0; bolt < numBolts; bolt++) {
                double boltPhase = (currentTime * 8.0 + bolt * 0.25) % 1.0;
                int boltPos = (int)(boltPhase * center);
                int trailLen = 5;
                
                for (int t = 0; t < trailLen; t++) {
                    int pos = boltPos - t;
                    if (pos >= 0 && pos < center) {
                        double fade = 1.0 - ((double) t / trailLen);
                        fade = fade * fade;
                        
                        // Gold-to-green gradient along trail
                        int[] boltColor = (t < 2) ? gold : green;
                        
                        blendAdditive(stripStart + center - pos, boltColor, fade * 0.9);
                        blendAdditive(stripStart + center + pos, boltColor, fade * 0.9);
                    }
                }
            }
            
            // Bright center core (always visible between flashes)
            double corePulse = (Math.sin(currentTime * Math.PI * 12) + 1) / 2 * 0.3 + 0.7;
            for (int c = -1; c <= 1; c++) {
                int idx = center + c;
                if (idx >= 0 && idx < stripCount) {
                    blendAdditive(stripStart + idx, Constants.LEDs.WARM_WHITE, 
                                  corePulse * (1.0 - Math.abs(c) * 0.3) * masterBrightness);
                }
            }
        }
        
        pushBuffer();
    }
    
    /**
     * AUTO MODE PATTERN - Fast scanning beam with sparkle accents.
     * A bright alliance-colored beam sweeps rapidly back and forth across the
     * strip like a scanner, with a white-hot leading edge and long phosphor
     * trail. Subtle sparkle accents appear randomly for visual interest.
     * The fast, precise motion conveys "autonomous — robot is in control."
     */
    private void setAutoModePattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        clearBuffer();
        
        // Onboard: fast alliance pulse
        double onboardPulse = (Math.sin(currentTime * Math.PI * 4) + 1) / 2 * 0.6 + 0.4;
        setOnboardColor(scaleColor(allianceColor, onboardPulse * masterBrightness));
        
        // Dim alliance base
        for (int i = 0; i < stripCount; i++) {
            setLED(stripStart + i, scaleColor(allianceColor, 0.04 * masterBrightness));
        }
        
        // === Scanning beam: bounces back and forth ===
        // Triangle wave for smooth back-and-forth (1.2 second cycle)
        double scanPhase = (currentTime % 1.2) / 1.2;
        double scanPos;
        if (scanPhase < 0.5) {
            scanPos = scanPhase * 2.0;  // 0→1 (forward sweep)
        } else {
            scanPos = 2.0 - scanPhase * 2.0;  // 1→0 (reverse sweep)
        }
        int beamHead = (int)(scanPos * (stripCount - 1));
        
        int trailLen = 12;
        int direction = (scanPhase < 0.5) ? -1 : 1;  // Trail goes opposite to movement
        
        for (int t = 0; t < trailLen; t++) {
            int idx = beamHead + t * direction;
            if (idx >= 0 && idx < stripCount) {
                double fade = 1.0 - ((double) t / trailLen);
                fade = fade * fade;
                
                if (t == 0) {
                    // White-hot leading edge
                    int[] tipColor = lerpColor(allianceColor, Constants.LEDs.WHITE, 0.7);
                    blendAdditive(stripStart + idx, tipColor, masterBrightness);
                } else if (t < 3) {
                    // Bright alliance near the head
                    int[] nearColor = lerpColor(allianceColor, Constants.LEDs.WARM_WHITE, 0.3 * fade);
                    blendAdditive(stripStart + idx, nearColor, fade * masterBrightness);
                } else {
                    // Fading phosphor trail
                    blendAdditive(stripStart + idx, allianceColor, fade * 0.8 * masterBrightness);
                }
            }
        }
        
        // === Subtle sparkle accents (3 at any time) ===
        for (int s = 0; s < 3; s++) {
            double sparkle = Math.sin(currentTime * 11.0 + s * 3.7);
            if (sparkle > 0.7) {
                double sparkBright = (sparkle - 0.7) / 0.3;
                sparkBright = sparkBright * sparkBright * 0.4;
                int sparkIdx = (int)(((long)(currentTime * 7 + s * 19) * 31) % stripCount);
                if (sparkIdx < 0) sparkIdx += stripCount;
                blendAdditive(stripStart + sparkIdx, Constants.LEDs.WHITE, sparkBright * masterBrightness);
            }
        }
        
        pushBuffer();
    }
    
    /**
     * TELEOP MODE PATTERN - Flowing gradient wave with organic shimmer.
     * A smooth lava-lamp style flow of alliance color in varying intensity,
     * with two sine waves at different speeds creating organic "liquid"
     * movement. A subtle white shimmer point drifts slowly across for
     * visual polish. Calm but clearly active — "human is driving."
     */
    private void setTeleopModePattern() {
        double currentTime = Timer.getFPGATimestamp();
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        clearBuffer();
        setOnboardBreathing(allianceColor);
        
        // Two flowing sine waves for organic liquid movement
        double drift1 = currentTime * 0.35;
        double drift2 = currentTime * 0.2;
        
        for (int i = 0; i < stripCount; i++) {
            double pos = (double) i / stripCount;
            
            // Layered sine waves with different wavelengths
            double wave1 = (Math.sin((pos * 3.0 - drift1) * Math.PI * 2) + 1) / 2;
            double wave2 = (Math.sin((pos * 2.0 + drift2) * Math.PI * 2) + 1) / 2;
            
            // Smooth blending with warm bias
            double combined = wave1 * wave1 * 0.6 + wave2 * wave2 * 0.4;
            double brightness = combined * 0.85 + 0.15;  // 15% - 100%
            
            // Subtle color variation: slightly warmer at peaks
            int[] pixelColor = lerpColor(allianceColor, 
                Constants.LEDs.WARM_WHITE, combined * 0.08);
            setLED(stripStart + i, scaleColor(pixelColor, brightness * masterBrightness));
        }
        
        // Drifting white shimmer point
        double shimmerPos = ((Math.sin(currentTime * 0.5) + 1) / 2) * stripCount;
        int shimmerIdx = (int) shimmerPos;
        double shimmerBright = (Math.sin(currentTime * 1.1) + 1) / 2 * 0.25;
        
        for (int s = -2; s <= 2; s++) {
            int idx = shimmerIdx + s;
            if (idx >= 0 && idx < stripCount) {
                double sFade = 1.0 - Math.abs(s) / 3.0;
                blendAdditive(stripStart + idx, Constants.LEDs.WARM_WHITE, 
                              shimmerBright * sFade * masterBrightness);
            }
        }
        
        pushBuffer();
    }
    
    /**
     * Updates LED pattern based on current state.
     * Sets currentPatternName for dashboard visibility.
     * checkEStop() is already called in periodic() and is not duplicated here.
     */
    private void updateLEDPattern() {
        double timeSinceStateStart = Timer.getFPGATimestamp() - stateStartTime;
        
        // Brownout is highest priority - always check first
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
        
        // PRIORITY 2: Actively firing - Override everything (driver needs instant feedback!)
        if (currentAction == ActionState.FIRING) {
            currentPatternName = "FIRING!";
            setFiringPattern();
            return;
        }
        
        // PRIORITY 3: Ready to shoot - driver needs to know they can fire
        if (currentAction == ActionState.SHOOTING) {
            currentPatternName = "SHOOT NOW";
            setShootNowPattern();
            return;
        }
        
        // Get game state info for time-based warnings
        double matchTime = DriverStation.getMatchTime();
        GamePhase phase = gameState.getGamePhase();
        boolean inEndgame = (matchTime > 0 && matchTime <= 30.0);
        
        // PRIORITY 4: Endgame warnings and urgency pattern
        // Endgame takes priority over internal actions (aiming, spooling, etc.)
        // because the driver needs to know time is running out regardless of what
        // the robot is doing. FIRING/SHOOTING still override since that's active scoring.
        if (currentState == LEDState.TELEOP && !inEndgame && matchTime > 0) {
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
        
        // Once in endgame phase, show endgame urgency pattern
        if (currentState == LEDState.TELEOP && phase == GamePhase.END_GAME) {
            currentPatternName = "Endgame Urgency";
            setEndgameUrgencyPattern();
            return;
        }
        
        // PRIORITY 5: Internal robot actions (aiming, spooling, climbing, intaking)
        // These show what the robot is actively doing but are less critical than time warnings
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
                    // Fall through to check shift warnings
                    break;
                    
                case FIRING:
                case SHOOTING:
                case ESTOP:
                case BROWNOUT:
                    return;
            }
        }
        
        // PRIORITY 6: Alliance shift warnings (only during TELEOP and when idle/driving)
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
                // Still in minimum boot window; do not send any commands
                DashboardHelper.putString(Category.DEBUG, "LED/Pattern", "Waiting for CANdle (" 
                    + String.format("%.1f", timeSinceBoot) + "s)");
                DashboardHelper.putString(Category.DEBUG, "LED/State", currentState.name());
                DashboardHelper.putString(Category.DEBUG, "LED/Action", currentAction.name());
                return;
            }
            // Minimum boot time elapsed; apply configuration
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
        
        // --- Pattern Dispatch (throttled to ~10Hz to reduce CAN traffic) ---
        ledUpdateCounter++;
        if (ledUpdateCounter >= LED_UPDATE_DIVISOR) {
            ledUpdateCounter = 0;
            updateLEDPattern();
        }
        
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
