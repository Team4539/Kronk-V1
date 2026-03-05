package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
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
 * LED subsystem using CTRE CANdle — SIMPLIFIED for clarity.
 * 
 * DESIGN PHILOSOPHY: Every state looks completely different at a glance.
 * Simple patterns that are easy to distinguish even through smoke/panels.
 * Only IDLE/DISABLED uses complex animations (robot is disabled, looks cool).
 * 
 * PHYSICAL LAYOUT (Y-spliced two-strip configuration):
 *   Indices 0-7:   Onboard LEDs (belly pan)
 *   Indices 8-45:  SHARED ZONE — visible on BOTH top and belly (38 LEDs)
 *   Indices 46-65: BELLY-ONLY ZONE — only on belly strip (20 LEDs)
 * 
 * STATE SUMMARY (each is instantly recognizable):
 *   DISABLED/IDLE  = Scrolling orange/blue blocks (clean, looks great in pits)
 *   AUTO/TRANSITION = Half red, half blue cycling back and forth (2 Hz)
 *   TELEOP         = Alliance color flowing comet (speeds up as shift ends)
 *   SPOOLING       = Orange breathing pulse ("warming up")
 *   READY TO SHOOT = Solid bright GREEN ("PULL THE TRIGGER!")
 *   FIRING         = Fast white strobe ("balls are flying!")
 *   INTAKING       = Solid yellow-green
 *   E-STOP         = Red scanner sweep + pulse (dramatic, robot is disabled)
 *   ENDGAME        = Half red, half blue fast cycling (5 Hz — urgent transition look)
 *   HEAD BACK      = Breathing white/alliance ("get to position")
 *   GREEN LIGHT    = Fast cyan/white strobe ("GO GO GO!")
 *   BROWNOUT       = Dim amber flicker
 *   MATCH END      = Victory gold/team color celebration
 */
public class LEDSubsystem extends SubsystemBase {
    
    // Hardware
    private final CANdle candle;
    private final CANdleConfiguration candleConfig;
    private final SolidColor solidColorRequest;
    
    // Per-LED color buffer for addressable animations
    private final int[][] ledBuffer;
    private final int ledCount;
    
    // Master brightness
    private double masterBrightness = 1.0;
    
    // CANdle boot readiness
    private boolean candleReady = false;
    private double bootStartTime = 0;
    private static final double CANDLE_MIN_BOOT_SECONDS = 1.5;
    
    // Current pattern name for dashboard
    private String currentPatternName = "Waiting for CANdle...";
    
    // State tracking
    private LEDState currentState = LEDState.BOOT_WARMUP;
    private ActionState currentAction = ActionState.IDLE;
    private double stateStartTime = 0;
    private int[] allianceColor = Constants.LEDs.BLUE_ALLIANCE;
    private boolean isEStopped = false;
    private boolean matchWasActive = false;
    private double matchEndCelebrationDuration = 10.0;
    
    // Game state manager
    private final GameStateManager gameState;
    
    // Notification state
    private boolean hasNotifiedEStop = false;
    private boolean hasNotifiedEndgame = false;
    
    // CAN frame deduplication
    private int lastSentR = -1, lastSentG = -1, lastSentB = -1;
    
    // LED update throttle: 10Hz
    private int ledUpdateCounter = 0;
    private static final int LED_UPDATE_DIVISOR = 5;
    
    // =========================================================================
    // ENUMS
    // =========================================================================
    
    /**
     * LED states for robot operation phases.
     */
    public enum LEDState {
        BOOT_WARMUP,    // Team colors animation while CANdle warms up
        DISABLED,       // Scrolling orange/blue blocks (clean, pits)
        AUTO,           // Red/blue half-and-half cycling
        TELEOP,         // Alliance color flow (accelerates as shift ends)
        ENDGAME,        // Red/blue fast cycle (like transition but urgent)
        MATCH_END,      // Victory celebration
        BROWNOUT        // Dim amber
    }
    
    /**
     * Robot action states — simple, distinct visual feedback.
     * Higher-priority actions override lower ones in the pattern dispatch.
     */
    public enum ActionState {
        IDLE,       // No special action — show base state pattern
        FIRING,     // Actively feeding balls — fast white strobe
        SHOOTING,   // Ready to shoot (spun up + aimed) — solid bright GREEN
        SPOOLING,   // Spinning up — orange breathing
        INTAKING,   // Running intake — solid yellow-green
        ESTOP,      // Emergency stop — red scanner sweep (dramatic, robot disabled)
        BROWNOUT    // Low voltage — amber flicker
    }
    
    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    
    public LEDSubsystem() {
        candle = new CANdle(Constants.CANIds.CANDLE);
        solidColorRequest = new SolidColor(0, Constants.LEDs.LED_COUNT - 1);
        gameState = GameStateManager.getInstance();
        
        ledCount = Constants.LEDs.LED_COUNT;
        ledBuffer = new int[ledCount][3];
        
        candleConfig = new CANdleConfiguration();
        candleConfig.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
        candleConfig.FutureProofConfigs = true;
        candleConfig.LED.StripType = StripTypeValue.GRB;
        
        bootStartTime = Timer.getFPGATimestamp();
        stateStartTime = bootStartTime;
        
        DashboardHelper.putNumber(Category.DEBUG, "LED/Master_Brightness", masterBrightness);
    }
    
    // =========================================================================
    // PUBLIC STATE CONTROL
    // =========================================================================
    
    /**
     * Sets the LED state. Detects match end and auto-triggers celebration.
     */
    public void setState(LEDState newState) {
        if (currentState != newState) {
            boolean wasInMatch = (currentState == LEDState.AUTO || currentState == LEDState.TELEOP
                                  || currentState == LEDState.ENDGAME);
            boolean goingToDisabled = (newState == LEDState.DISABLED);
            
            if (wasInMatch && goingToDisabled && matchWasActive) {
                newState = LEDState.MATCH_END;
                matchWasActive = false;
            }
            if (newState == LEDState.AUTO || newState == LEDState.TELEOP) {
                matchWasActive = true;
            }
            
            currentState = newState;
            stateStartTime = Timer.getFPGATimestamp();
            // Force CAN update on state change
            lastSentR = -1; lastSentG = -1; lastSentB = -1;
        }
    }
    
    public LEDState getState() { return currentState; }
    
    public void setAction(ActionState action) { currentAction = action; }
    public ActionState getAction() { return currentAction; }
    public void clearAction() { currentAction = ActionState.IDLE; }
    
    // =========================================================================
    // CORE LED OUTPUT METHODS
    // =========================================================================
    
    /**
     * Sets ALL LEDs to a solid color. Deduplicates CAN frames.
     */
    private void setSolidColor(int[] color) {
        int r = (int)(color[0] * masterBrightness);
        int g = (int)(color[1] * masterBrightness);
        int b = (int)(color[2] * masterBrightness);
        if (r == lastSentR && g == lastSentG && b == lastSentB) return;
        lastSentR = r;
        lastSentG = g;
        lastSentB = b;
        candle.setControl(solidColorRequest.withColor(new RGBWColor(r, g, b, 0)));
    }
    
    /** Sets a single LED in the buffer (call pushBuffer() to send to hardware). */
    private void setLED(int index, int[] color) {
        if (index >= 0 && index < ledCount) {
            ledBuffer[index][0] = (int)(color[0] * masterBrightness);
            ledBuffer[index][1] = (int)(color[1] * masterBrightness);
            ledBuffer[index][2] = (int)(color[2] * masterBrightness);
        }
    }
    
    /** Clears the LED buffer (all off). */
    private void clearBuffer() {
        for (int i = 0; i < ledCount; i++) {
            ledBuffer[i][0] = 0;
            ledBuffer[i][1] = 0;
            ledBuffer[i][2] = 0;
        }
    }
    
    /** Pushes the LED buffer to hardware using efficient segment grouping. */
    private void pushBuffer() {
        // Invalidate dedup cache since we're using per-LED control
        lastSentR = -1; lastSentG = -1; lastSentB = -1;
        
        int i = 0;
        while (i < ledCount) {
            int startIdx = i;
            int r = ledBuffer[i][0];
            int g = ledBuffer[i][1];
            int b = ledBuffer[i][2];
            while (i < ledCount && ledBuffer[i][0] == r && ledBuffer[i][1] == g && ledBuffer[i][2] == b) {
                i++;
            }
            candle.setControl(new SolidColor(startIdx, i - 1).withColor(new RGBWColor(r, g, b, 0)));
        }
    }
    
    // =========================================================================
    // COLOR UTILITIES
    // =========================================================================
    
    private int[] lerpColor(int[] a, int[] b, double t) {
        t = Math.max(0.0, Math.min(1.0, t));
        return new int[] {
            (int)(a[0] + (b[0] - a[0]) * t),
            (int)(a[1] + (b[1] - a[1]) * t),
            (int)(a[2] + (b[2] - a[2]) * t)
        };
    }
    
    private int[] scaleColor(int[] color, double factor) {
        return new int[] {
            Math.min(255, Math.max(0, (int)(color[0] * factor))),
            Math.min(255, Math.max(0, (int)(color[1] * factor))),
            Math.min(255, Math.max(0, (int)(color[2] * factor)))
        };
    }
    
    /** 1D pseudo-noise for organic animations (only used by IDLE aurora). */
    private double noise1D(double x) {
        double v = Math.sin(x * 1.0) * 0.5
                 + Math.sin(x * 2.3 + 1.7) * 0.25
                 + Math.sin(x * 5.1 + 3.1) * 0.125;
        return (v / 0.875 + 1.0) / 2.0;
    }
    
    // =========================================================================
    // SIMPLE ANIMATION PRIMITIVES
    // =========================================================================
    
    /**
     * Breathing/pulse effect. Smooth sine wave brightness modulation.
     * @param color  Base color
     * @param cycleSeconds  Duration of one full pulse cycle
     */
    private void setBreathing(int[] color, double cycleSeconds) {
        double phase = (Timer.getFPGATimestamp() % cycleSeconds) / cycleSeconds;
        double brightness = (Math.sin(phase * Math.PI * 2) + 1) / 2;
        brightness = 0.15 + 0.85 * brightness; // Floor at 15% so it never fully darkens
        setSolidColor(scaleColor(color, brightness));
    }
    
    /**
     * Strobe — on/off flashing at given Hz.
     */
    private void setStrobe(int[] color, double hz) {
        boolean isOn = ((int)(Timer.getFPGATimestamp() * hz) % 2) == 0;
        setSolidColor(isOn ? color : Constants.LEDs.OFF);
    }
    
    /**
     * Two-color alternating strobe at given Hz.
     */
    private void setTwoColorStrobe(int[] color1, int[] color2, double hz) {
        boolean showFirst = ((int)(Timer.getFPGATimestamp() * hz) % 2) == 0;
        setSolidColor(showFirst ? color1 : color2);
    }
    
    // =========================================================================
    // PATTERN: AUTO / TRANSITION — Red/Blue half-and-half cycling
    // =========================================================================
    
    /**
     * AUTO pattern: Left half red, right half blue (or swapped).
     * Swaps sides at the given rate for a striking cycling effect.
     * Used during autonomous AND the ~3s auto-to-teleop transition (2 Hz),
     * and again during endgame at a faster rate (5 Hz) for urgency.
     * 
     * @param swapHz  How many times per second the halves swap (e.g. 2.0 = swap every 0.5s)
     */
    private void setRedBlueCyclePattern(double swapHz) {
        double currentTime = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int halfPoint = stripCount / 2;
        
        // Swap sides based on Hz
        boolean swapped = ((int)(currentTime * swapHz) % 2) == 0;
        
        int[] leftColor  = swapped ? Constants.LEDs.RED_ALLIANCE : Constants.LEDs.BLUE_ALLIANCE;
        int[] rightColor = swapped ? Constants.LEDs.BLUE_ALLIANCE : Constants.LEDs.RED_ALLIANCE;
        
        clearBuffer();
        
        // Onboard LEDs: split red/blue
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            int[] c = (i < Constants.LEDs.ONBOARD_LED_COUNT / 2) ? leftColor : rightColor;
            ledBuffer[i][0] = (int)(c[0] * masterBrightness);
            ledBuffer[i][1] = (int)(c[1] * masterBrightness);
            ledBuffer[i][2] = (int)(c[2] * masterBrightness);
        }
        
        // Strip: left half one color, right half the other
        for (int i = 0; i < stripCount; i++) {
            setLED(stripStart + i, (i < halfPoint) ? leftColor : rightColor);
        }
        
        pushBuffer();
    }
    
    // =========================================================================
    // PATTERN: IDLE / DISABLED — Scrolling orange-blue blocks
    // =========================================================================
    
    /**
     * Clean scrolling blocks of team orange and blue. Each block is a distinct
     * solid color with a short (1-2 LED) fade between them — no messy blending.
     * Gentle brightness breathing keeps it alive without looking "puked together".
     */
    private void setIdleAuroraPattern() {
        double currentTime = Timer.getFPGATimestamp();
        int[] orange = Constants.LEDs.TEAM_SAFETY_ORANGE;
        int[] blue = Constants.LEDs.TEAM_BLUE;
        int onboard = Constants.LEDs.ONBOARD_LED_COUNT;
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        // Block size: how many LEDs per color block
        int blockSize = 8;
        // Fade zone: 2 LEDs between blocks for a clean transition
        int fadeZone = 2;
        int fullBlock = blockSize + fadeZone; // one color block + its fade
        
        // Scroll offset: moves smoothly along the strip
        double scrollSpeed = 4.0; // LEDs per second
        double scrollOffset = currentTime * scrollSpeed;
        
        // Gentle global breathing so it's not static
        double breath = 0.6 + 0.4 * ((Math.sin(currentTime * 0.8) + 1.0) / 2.0);
        
        clearBuffer();
        
        // Onboard LEDs: alternating solid orange/blue, no blending
        for (int i = 0; i < onboard && i < ledCount; i++) {
            int[] color = (i < onboard / 2) ? orange : blue;
            ledBuffer[i][0] = (int)(color[0] * breath * masterBrightness);
            ledBuffer[i][1] = (int)(color[1] * breath * masterBrightness);
            ledBuffer[i][2] = (int)(color[2] * breath * masterBrightness);
        }
        
        // Strip: scrolling blocks with short fade transitions
        for (int i = 0; i < stripCount; i++) {
            // Where does this LED fall in the repeating block pattern?
            double pos = (i + scrollOffset) % (fullBlock * 2);
            if (pos < 0) pos += fullBlock * 2;
            
            int[] color;
            if (pos < blockSize) {
                // Solid orange block
                color = orange;
            } else if (pos < blockSize + fadeZone) {
                // Short fade: orange → blue
                double t = (pos - blockSize) / fadeZone;
                color = lerpColor(orange, blue, t);
            } else if (pos < blockSize + fadeZone + blockSize) {
                // Solid blue block
                color = blue;
            } else {
                // Short fade: blue → orange
                double t = (pos - blockSize - fadeZone - blockSize) / fadeZone;
                color = lerpColor(blue, orange, t);
            }
            
            setLED(stripStart + i, scaleColor(color, breath));
        }
        
        pushBuffer();
    }
    
    // =========================================================================
    // PATTERN: VICTORY CELEBRATION — Gold/team color sparkle
    // =========================================================================
    
    private void setVictoryCelebration() {
        double currentTime = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        int[] gold = Constants.LEDs.VICTORY_GOLD;
        int[] orange = Constants.LEDs.TEAM_SAFETY_ORANGE;
        int[] blue = Constants.LEDs.TEAM_BLUE;
        int[] white = Constants.LEDs.WHITE;
        
        clearBuffer();
        
        // Onboard: rapid gold/white/blue cycle
        int onboardCycle = ((int)(currentTime * 8)) % 3;
        int[] onboardColor = (onboardCycle == 0) ? gold : (onboardCycle == 1) ? white : blue;
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            ledBuffer[i][0] = (int)(onboardColor[0] * masterBrightness);
            ledBuffer[i][1] = (int)(onboardColor[1] * masterBrightness);
            ledBuffer[i][2] = (int)(onboardColor[2] * masterBrightness);
        }
        
        // Strip: sparkle rain in team colors
        for (int i = 0; i < stripCount; i++) {
            double twinkle = noise1D(i * 0.7 + currentTime * 2.5);
            twinkle = twinkle * twinkle;
            double colorCycle = noise1D(i * 0.3 + currentTime * 0.4);
            int[] baseColor;
            if (colorCycle > 0.6) baseColor = gold;
            else if (colorCycle > 0.3) baseColor = orange;
            else baseColor = blue;
            double brightness = 0.2 + 0.8 * twinkle;
            setLED(stripStart + i, scaleColor(baseColor, brightness));
        }
        
        pushBuffer();
    }
    
    // =========================================================================
    // PATTERN: BROWNOUT — Dim amber flicker
    // =========================================================================
    
    private void setBrownoutPattern() {
        double currentTime = Timer.getFPGATimestamp();
        double flicker = 0.1 + 0.3 * ((Math.sin(currentTime * 6.0) + 1) / 2)
                        * ((Math.sin(currentTime * 8.7 + 1.3) + 1) / 2);
        setSolidColor(scaleColor(Constants.LEDs.BROWNOUT_SPARK, flicker));
    }
    
    // =========================================================================
    // PATTERN: E-STOP — Dramatic red scanner + pulse (robot is disabled)
    // =========================================================================
    
    /**
     * Dramatic e-stop pattern. Since the robot is disabled anyway, we can make
     * this look cool: a red scanner/larson sweep with a pulsing background.
     * Clearly communicates "something is wrong" while looking intentional.
     */
    private void setEStopPattern() {
        double currentTime = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int[] red = Constants.LEDs.ESTOP_COLOR;
        
        // Background: slow dim red pulse so it never looks "off"
        double bgPulse = 0.05 + 0.10 * ((Math.sin(currentTime * 3.0) + 1) / 2);
        int[] dimRed = scaleColor(red, bgPulse);
        
        // Scanner position: bounces back and forth at ~1.5 Hz
        double scanPhase = (currentTime * 1.5) % 1.0;
        double scanPos = scanPhase < 0.5 ? (scanPhase * 2.0) : (1.0 - (scanPhase - 0.5) * 2.0);
        double headIndex = scanPos * (stripCount - 1);
        int scanWidth = Math.max(3, stripCount / 8); // ~12% of strip
        
        clearBuffer();
        
        // Onboard LEDs: pulsing red
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            double onboardPulse = 0.3 + 0.7 * ((Math.sin(currentTime * 4.0) + 1) / 2);
            int[] c = scaleColor(red, onboardPulse);
            ledBuffer[i][0] = (int)(c[0] * masterBrightness);
            ledBuffer[i][1] = (int)(c[1] * masterBrightness);
            ledBuffer[i][2] = (int)(c[2] * masterBrightness);
        }
        
        // Strip: scanner with glow
        for (int i = 0; i < stripCount; i++) {
            double dist = Math.abs(i - headIndex);
            double brightness;
            if (dist <= 1.0) {
                brightness = 1.0; // Scanner head — full bright
            } else if (dist <= scanWidth) {
                double falloff = 1.0 - (dist / scanWidth);
                brightness = falloff * falloff; // Quadratic glow tail
            } else {
                brightness = 0.0;
            }
            
            if (brightness > 0.01) {
                setLED(stripStart + i, scaleColor(red, brightness));
            } else {
                setLED(stripStart + i, dimRed);
            }
        }
        
        pushBuffer();
    }
    
    // =========================================================================
    // PATTERN: HEAD BACK WARNING — Breathing white/alliance blend
    // =========================================================================
    
    /**
     * Breathing blend between alliance color and white.
     * Simple "get to position" visual cue.
     */
    private void setHeadBackWarning() {
        double currentTime = Timer.getFPGATimestamp();
        double phase = (currentTime % 1.0) / 1.0;
        double brightness = (Math.sin(phase * Math.PI * 2) + 1) / 2;
        int[] color = lerpColor(allianceColor, Constants.LEDs.WHITE, brightness);
        setSolidColor(scaleColor(color, 0.5 + 0.5 * brightness));
    }
    
    // =========================================================================
    // PATTERN: GREEN LIGHT PRE-SHIFT — Fast cyan/white strobe
    // =========================================================================
    
    private void setGreenLightPreShift() {
        setTwoColorStrobe(Constants.LEDs.CYAN, Constants.LEDs.WHITE, 10.0);
    }
    
    // =========================================================================
    // PATTERN: TELEOP FLOW — Accelerating comet that speeds up as shift ends
    // =========================================================================
    
    /**
     * Flowing comet pattern in the active alliance color. Speed increases as the
     * shift progresses, creating a visual sense of urgency.
     * 
     * @param shiftProgress 0.0 = shift just started (slow sweep), 1.0 = about to end (fast).
     *                      If negative, uses a relaxed default speed (no FMS data).
     */
    private void setTeleopFlowPattern(double shiftProgress) {
        double currentTime = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        int[] color = getTeleopDisplayColor();
        
        // Speed ramps from slow to fast as shift progresses.
        // sweepHz = full sweeps per second. Starts at 0.3 (3.3s per sweep), ends at 3.0 (0.33s).
        double sweepHz;
        if (shiftProgress < 0) {
            // No FMS data — use a gentle default flow
            sweepHz = 0.4;
        } else {
            // Exponential ramp feels more natural than linear: slow start, rapid finish
            // progress 0.0 → 0.3 Hz, progress 1.0 → 3.0 Hz
            sweepHz = 0.3 + 2.7 * shiftProgress * shiftProgress;
        }
        
        // Position of comet head along the strip [0, 1), bouncing back and forth
        double rawPhase = (currentTime * sweepHz) % 1.0;
        // Triangle wave: 0→1→0 for smooth bounce
        double cometPos = rawPhase < 0.5 ? (rawPhase * 2.0) : (1.0 - (rawPhase - 0.5) * 2.0);
        
        // Comet parameters
        int cometLength = Math.max(4, stripCount / 5); // ~20% of strip is lit
        double headIndex = cometPos * (stripCount - 1);
        
        // Dim background: 8% of alliance color so it's never fully dark
        int[] dimBg = scaleColor(color, 0.08);
        
        clearBuffer();
        
        // Onboard LEDs: solid dim alliance color
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            ledBuffer[i][0] = (int)(dimBg[0] * masterBrightness);
            ledBuffer[i][1] = (int)(dimBg[1] * masterBrightness);
            ledBuffer[i][2] = (int)(dimBg[2] * masterBrightness);
        }
        
        // Strip: comet with tail fade
        for (int i = 0; i < stripCount; i++) {
            double dist = Math.abs(i - headIndex);
            double brightness;
            if (dist <= 1.0) {
                // Comet head — full brightness
                brightness = 1.0;
            } else if (dist <= cometLength) {
                // Tail — fades out with distance from head
                double tailFactor = 1.0 - (dist / cometLength);
                brightness = tailFactor * tailFactor; // Quadratic falloff for nice tail shape
            } else {
                // Background
                brightness = 0.0;
            }
            
            // Blend between dim bg and full color
            if (brightness > 0.01) {
                int[] pixel = lerpColor(dimBg, color, brightness);
                setLED(stripStart + i, pixel);
            } else {
                setLED(stripStart + i, dimBg);
            }
        }
        
        pushBuffer();
    }
    
    // =========================================================================
    // TELEOP DISPLAY COLOR
    // =========================================================================
    
    /**
     * Gets the color to display during teleop:
     * - If we know which alliance is currently active from FMS, show THAT color
     * - Otherwise fall back to our own alliance color
     */
    private int[] getTeleopDisplayColor() {
        Alliance active = gameState.getCurrentlyActiveAlliance();
        if (active != null) {
            return (active == Alliance.Red) ? Constants.LEDs.RED_ALLIANCE : Constants.LEDs.BLUE_ALLIANCE;
        }
        return allianceColor;
    }
    
    // =========================================================================
    // MAIN PATTERN DISPATCH — priority-based, simple and clear
    // =========================================================================
    
    /**
     * Updates LED pattern based on current state and action.
     * 
     * Priority order (highest first):
     *  1. Brownout (safety)
     *  2. E-Stop (safety)
     *  3. Firing (active scoring feedback)
     *  4. Ready to Shoot (green = pull trigger)
     *  5. Spooling (orange breathing = warming up)
     *  6. Intaking (solid yellow-green)
     *  7. Match phase patterns (auto cycle, teleop, endgame, etc.)
     */
    private void updateLEDPattern() {
        double timeSinceStateStart = Timer.getFPGATimestamp() - stateStartTime;
        double matchTime = DriverStation.getMatchTime();
        GamePhase phase = gameState.getGamePhase();
        
        // === PRIORITY 1: BROWNOUT (safety) ===
        if (currentAction == ActionState.BROWNOUT) {
            currentPatternName = "Brownout";
            setBrownoutPattern();
            return;
        }
        
        // === PRIORITY 2: E-STOP (safety — robot is disabled, look dramatic) ===
        if (isEStopped || currentAction == ActionState.ESTOP) {
            currentPatternName = "E-STOP";
            setEStopPattern();
            return;
        }
        
        // === PRIORITY 3: FIRING — balls are actively being fed ===
        if (currentAction == ActionState.FIRING) {
            currentPatternName = "FIRING!";
            setStrobe(Constants.LEDs.WHITE, 15.0);
            return;
        }
        
        // === PRIORITY 4: READY TO SHOOT — solid GREEN ===
        // This is the critical "you can pull the trigger now" indicator.
        // Set by RobotContainer when shooter.isReady() during LT spool.
        if (currentAction == ActionState.SHOOTING) {
            currentPatternName = "READY - SHOOT!";
            setSolidColor(Constants.LEDs.SHOOTING_COLOR);
            return;
        }
        
        // === PRIORITY 5: SPOOLING — orange breathing ===
        if (currentAction == ActionState.SPOOLING) {
            currentPatternName = "Spooling Up";
            setBreathing(Constants.LEDs.SPOOLING_COLOR, 1.0);
            return;
        }
        
        // === PRIORITY 6: INTAKING — solid yellow-green ===
        if (currentAction == ActionState.INTAKING) {
            currentPatternName = "Intaking";
            setSolidColor(Constants.LEDs.INTAKING_COLOR);
            return;
        }
        
        // === PRIORITY 7: Match phase patterns ===
        switch (currentState) {
            case BOOT_WARMUP:
                currentPatternName = "Boot Warmup";
                setIdleAuroraPattern();
                if (timeSinceStateStart > Constants.LEDs.BOOT_ANIMATION_DURATION) {
                    setState(LEDState.DISABLED);
                }
                break;
                
            case DISABLED:
                currentPatternName = "Disabled (Blocks)";
                setIdleAuroraPattern();
                break;
                
            case AUTO:
                // Red/blue half-and-half cycling.
                // Continues through the ~3s auto-to-teleop transition because
                // Robot.java keeps the AUTO state during that gap.
                // Switches to TELEOP when teleopInit() fires.
                currentPatternName = "Auto (Red/Blue Cycle)";
                setRedBlueCyclePattern(2.0);
                break;
                
            case TELEOP:
                // Endgame (last 30 seconds) — fast red/blue cycle like transition but urgent
                if (phase == GamePhase.END_GAME) {
                    currentPatternName = "ENDGAME!";
                    setRedBlueCyclePattern(5.0);
                    break;
                }
                
                // Endgame countdown warnings (5 seconds before endgame)
                if (matchTime > 0) {
                    double timeUntilEndgame = matchTime - 30.0;
                    if (timeUntilEndgame > 0 && timeUntilEndgame <= 5.0) {
                        currentPatternName = "Endgame Warning";
                        setTwoColorStrobe(allianceColor, Constants.LEDs.WHITE, 4.0);
                        break;
                    }
                }
                
                // Shift timing warnings (only when FMS provides timing data)
                if (gameState.isHeadBackWarning()) {
                    currentPatternName = "Head Back!";
                    setHeadBackWarning();
                    break;
                }
                if (gameState.isGreenLightPreShift()) {
                    currentPatternName = "GREEN LIGHT - GO!";
                    setGreenLightPreShift();
                    break;
                }
                
                // Normal teleop: flowing comet that speeds up as shift ends
                currentPatternName = "Teleop (Flow)";
                setTeleopFlowPattern(gameState.getCurrentShiftProgress());
                break;
                
            case ENDGAME:
                currentPatternName = "ENDGAME!";
                setRedBlueCyclePattern(5.0);
                break;
                
            case MATCH_END:
                currentPatternName = "Victory!";
                setVictoryCelebration();
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
    
    // =========================================================================
    // PERIODIC
    // =========================================================================
    
    @Override
    public void periodic() {
        // --- CANdle boot gate ---
        if (!candleReady) {
            double timeSinceBoot = Timer.getFPGATimestamp() - bootStartTime;
            if (timeSinceBoot < CANDLE_MIN_BOOT_SECONDS) {
                DashboardHelper.putString(Category.DEBUG, "LED/Pattern",
                    "Waiting for CANdle (" + String.format("%.1f", timeSinceBoot) + "s)");
                DashboardHelper.putString(Category.DEBUG, "LED/State", currentState.name());
                DashboardHelper.putString(Category.DEBUG, "LED/Action", currentAction.name());
                return;
            }
            candle.getConfigurator().apply(candleConfig);
            candleReady = true;
            stateStartTime = Timer.getFPGATimestamp();
        }
        
        // --- Dashboard brightness control ---
        double newBrightness = DashboardHelper.getNumber(Category.DEBUG, "LED/Master_Brightness", masterBrightness);
        if (Math.abs(newBrightness - masterBrightness) > 0.01) {
            masterBrightness = Math.max(0.0, Math.min(1.0, newBrightness));
        }
        
        // --- Update alliance color ---
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            allianceColor = (alliance.get() == Alliance.Red)
                ? Constants.LEDs.RED_ALLIANCE : Constants.LEDs.BLUE_ALLIANCE;
        }
        
        // --- Check E-stop ---
        isEStopped = DriverStation.isEStopped();
        
        // --- Throttled pattern update (~10Hz) ---
        ledUpdateCounter++;
        if (ledUpdateCounter >= LED_UPDATE_DIVISOR) {
            ledUpdateCounter = 0;
            updateLEDPattern();
        }
        
        // --- Dashboard telemetry ---
        DashboardHelper.putString(Category.DEBUG, "LED/Pattern", currentPatternName);
        DashboardHelper.putString(Category.DEBUG, "LED/State", currentState.name());
        DashboardHelper.putString(Category.DEBUG, "LED/Action", currentAction.name());
        
        // --- Critical notifications ---
        double matchTime = DriverStation.getMatchTime();
        boolean inEndgame = (matchTime > 0 && matchTime <= 30.0);
        checkCriticalNotifications(matchTime, inEndgame);
    }
    
    /**
     * Sends critical notifications for E-stop and endgame.
     */
    private void checkCriticalNotifications(double matchTime, boolean inEndgame) {
        if (isEStopped && !hasNotifiedEStop) {
            hasNotifiedEStop = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.ERROR)
                .withTitle("E-STOP ACTIVATED!")
                .withDescription("Robot is emergency stopped")
                .withDisplaySeconds(10.0));
        } else if (!isEStopped) {
            hasNotifiedEStop = false;
        }
        
        if (inEndgame && !hasNotifiedEndgame && matchTime <= 30.0 && matchTime > 28.0) {
            hasNotifiedEndgame = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle("ENDGAME!")
                .withDescription("30 seconds remaining!")
                .withDisplaySeconds(3.0));
        }
        if (matchTime > 30.0 || matchTime < 0) {
            hasNotifiedEndgame = false;
        }
    }
    
    // =========================================================================
    // PUBLIC UTILITY METHODS
    // =========================================================================
    
    /** Trigger post-match victory celebration. */
    public void triggerMatchEndCelebration() { setState(LEDState.MATCH_END); }
    
    /** Solid team orange for pit display. */
    public void setTeamOrange() { setSolidColor(Constants.LEDs.TEAM_SAFETY_ORANGE); }
    
    /** Solid team blue for pit display. */
    public void setTeamBlue() { setSolidColor(Constants.LEDs.TEAM_BLUE); }
}
