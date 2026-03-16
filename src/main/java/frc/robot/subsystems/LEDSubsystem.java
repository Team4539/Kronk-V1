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
 * LED subsystem using CTRE CANdle with priority-based pattern dispatch.
 * 
 * DESIGN PHILOSOPHY: Every state looks completely different at a glance.
 * Simple patterns that are easy to distinguish even through smoke/panels.
 * Only IDLE/DISABLED uses complex animations (robot is disabled, looks cool).
 * 
 * PHYSICAL LAYOUT (back strip only — belly strip disconnected):
 *   Indices 0-7:   Onboard LEDs
 *   Indices 8-45:  Back/top strip (38 LEDs)
 * 
 * PATTERN SUMMARY (each is instantly recognizable):
 *   DISABLED/IDLE  = Scrolling orange/blue blocks with gentle breathing
 *   AUTO/TRANSITION = Half red, half blue cycling back and forth (2 Hz)
 *   TELEOP ACTIVE  = Alliance color progressive chase (tail fills as shift ends)
 *   TELEOP WAITING = Other alliance color chase, blends to ours in last 5s
 *   ENDGAME        = Alliance color chase starting at 50% urgency, ramps to full
 *   BROWNOUT       = Dim amber flicker (safety — highest priority)
 *   E-STOP         = EXCESSIVE dual red scanner + strobe + alternating onboard
 *   GAME DATA      = Announcement when FMS sends who goes first (3s, runs at 50Hz)
 *   FIRING         = Fast white strobe (15 Hz)
 *   READY TO SHOOT = Solid bright GREEN ("PULL THE TRIGGER!")
 *   SPOOLING       = Orange breathing pulse ("warming up")
 *   INTAKING       = Solid yellow-green
 *   FORCE SHOOT    = Purple breathing (force shoot override active)
 *   DEFENSIVE      = Pulsing amber/gold (brake mode active)
 *   MATCH END      = Victory gold/team color sparkle celebration
 * 
 * PROGRESSIVE CHASE: A dot races along the strip with a growing tail.
 * As urgency increases (0→1), the tail grows from ~5 LEDs to the full strip,
 * speed ramps from 0.4 to 2.0 strip/sec, and tail floor brightness rises.
 * At max urgency the strip is effectively solid color. Used for all TELEOP
 * states (active shift, waiting, endgame) with different colors and urgency
 * sources to create a unified visual language.
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
    
    // Game data announcement
    private boolean announcementActive = false;
    private double announcementStartTime = 0;
    private boolean announcementWeGoFirst = false;
    private static final double ANNOUNCEMENT_DURATION = 3.0; // seconds
    
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
     * TELEOP handles endgame internally via GamePhase — no separate ENDGAME state needed.
     */
    public enum LEDState {
        BOOT_WARMUP,    // Team colors aurora while CANdle warms up
        DISABLED,       // Scrolling orange/blue blocks (pits / pre-match)
        AUTO,           // Red/blue half-and-half cycling (2 Hz)
        TELEOP,         // Progressive chase — color and urgency vary by shift/endgame
        ENDGAME,        // (Unused — endgame handled inside TELEOP via GamePhase.END_GAME)
        MATCH_END,      // Victory gold/team color sparkle celebration
        BROWNOUT        // Dim amber flicker
    }
    
    /**
     * Robot action states — simple, distinct visual feedback.
     * Higher-priority actions override lower ones in the pattern dispatch.
     */
    public enum ActionState {
        IDLE,        // No special action — show base state pattern
        FIRING,      // Actively feeding balls — fast white strobe
        SHOOTING,    // Ready to shoot (spun up + aimed) — solid bright GREEN
        SPOOLING,    // Spinning up — orange breathing
        INTAKING,    // Running intake — solid yellow-green
        DEFENSIVE,   // Defensive brake mode — pulsing amber/gold
        FORCE_SHOOT, // Force shoot enabled — purple breathing
        BROWNOUT     // Low voltage — amber flicker
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
    // PATTERN: E-STOP — EXCESSIVE dramatic red scanner + strobe + pulse
    // =========================================================================
    
    /**
     * Over-the-top e-stop pattern. Red scanner sweep with a fast strobe overlay,
     * pulsing background, and alternating red/white flashes on the onboard LEDs.
     * This should scream "SOMETHING IS VERY WRONG" from across the field.
     */
    private void setEStopPattern() {
        double currentTime = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int[] red = Constants.LEDs.ESTOP_COLOR;
        int[] white = Constants.LEDs.WHITE;
        int[] hotCore = Constants.LEDs.ESTOP_CORE;
        
        // Fast strobe overlay — entire strip flashes red/white at 8 Hz
        boolean strobeOn = ((int)(currentTime * 8.0) % 2) == 0;
        
        // Background: aggressive red pulse (never fully off)
        double bgPulse = 0.10 + 0.25 * ((Math.sin(currentTime * 5.0) + 1) / 2);
        int[] dimRed = scaleColor(red, bgPulse);
        
        // Scanner: TWO heads bouncing in opposite directions at ~2 Hz
        double scanPhase1 = (currentTime * 2.0) % 1.0;
        double scanPos1 = scanPhase1 < 0.5 ? (scanPhase1 * 2.0) : (1.0 - (scanPhase1 - 0.5) * 2.0);
        double scanPhase2 = ((currentTime * 2.0) + 0.5) % 1.0;
        double scanPos2 = scanPhase2 < 0.5 ? (scanPhase2 * 2.0) : (1.0 - (scanPhase2 - 0.5) * 2.0);
        double head1 = scanPos1 * (stripCount - 1);
        double head2 = scanPos2 * (stripCount - 1);
        int scanWidth = Math.max(4, stripCount / 6);
        
        clearBuffer();
        
        // Onboard LEDs: alternating red/white rapid flash (different Hz than strip)
        boolean onboardFlash = ((int)(currentTime * 12.0) % 2) == 0;
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            // Even LEDs flash opposite to odd LEDs for chaotic look
            boolean thisOn = (i % 2 == 0) ? onboardFlash : !onboardFlash;
            int[] c = thisOn ? red : white;
            ledBuffer[i][0] = (int)(c[0] * masterBrightness);
            ledBuffer[i][1] = (int)(c[1] * masterBrightness);
            ledBuffer[i][2] = (int)(c[2] * masterBrightness);
        }
        
        // Strip: dual scanner with glow + strobe overlay
        for (int i = 0; i < stripCount; i++) {
            double dist1 = Math.abs(i - head1);
            double dist2 = Math.abs(i - head2);
            double minDist = Math.min(dist1, dist2);
            
            double brightness;
            if (minDist <= 1.0) {
                brightness = 1.0; // Scanner head — full bright
            } else if (minDist <= scanWidth) {
                double falloff = 1.0 - (minDist / scanWidth);
                brightness = falloff * falloff;
            } else {
                brightness = 0.0;
            }
            
            if (brightness > 0.5) {
                // Scanner head area — hot white-red core with strobe
                int[] headColor = strobeOn ? white : hotCore;
                setLED(stripStart + i, scaleColor(headColor, brightness));
            } else if (brightness > 0.01) {
                // Scanner glow tail
                setLED(stripStart + i, scaleColor(red, brightness));
            } else {
                // Background with strobe overlay
                if (strobeOn) {
                    setLED(stripStart + i, scaleColor(red, bgPulse * 1.5));
                } else {
                    setLED(stripStart + i, dimRed);
                }
            }
        }
        
        pushBuffer();
    }
    
    // =========================================================================
    // PATTERN: GAME DATA ANNOUNCEMENT — "WE KNOW WHO GOES FIRST!"
    // =========================================================================
    
    /**
     * Excessive announcement when game data arrives. The robot might know
     * before the drivers do — make it UNMISSABLE.
     * 
     * WE GO FIRST:  Rapid alliance-color explosion — scanner + strobe + pulse,
     *               building to a solid flash. "LET'S GOOO!"
     * THEY GO FIRST: Other alliance color rapid flash that fades into our
     *                alliance color breathing. "We're second — get ready."
     * 
     * @param elapsed  Seconds since announcement started (0 → ANNOUNCEMENT_DURATION)
     * @param weGoFirst  True if our alliance scores first
     */
    private void setGameDataAnnouncement(double elapsed, boolean weGoFirst) {
        double currentTime = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        // Progress through the announcement (0→1)
        double progress = Math.min(1.0, elapsed / ANNOUNCEMENT_DURATION);
        
        // Which color to celebrate with
        int[] primaryColor = weGoFirst ? allianceColor : getOtherAllianceColor();
        int[] secondaryColor = weGoFirst ? Constants.LEDs.WHITE : allianceColor;
        
        clearBuffer();
        
        if (weGoFirst) {
            // === WE GO FIRST — FULL CELEBRATION ===
            
            // Phase 1 (0-0.5): Rapid strobe between alliance color and white
            // Phase 2 (0.5-0.8): Scanner explosion outward from center
            // Phase 3 (0.8-1.0): Solid alliance color pulsing bright
            
            if (progress < 0.5) {
                // RAPID STROBE — 12Hz alliance/white alternating
                double strobeHz = 12.0;
                boolean showPrimary = ((int)(currentTime * strobeHz * 2) % 2) == 0;
                
                // Onboard: opposite phase for extra chaos
                for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
                    int[] c = ((i % 2 == 0) == showPrimary) ? primaryColor : secondaryColor;
                    ledBuffer[i][0] = (int)(c[0] * masterBrightness);
                    ledBuffer[i][1] = (int)(c[1] * masterBrightness);
                    ledBuffer[i][2] = (int)(c[2] * masterBrightness);
                }
                
                // Strip: alternating blocks that shift
                int blockSize = 3;
                int shift = (int)(currentTime * 30) % (blockSize * 2);
                for (int i = 0; i < stripCount; i++) {
                    boolean isPrimary = ((i + shift) / blockSize) % 2 == 0;
                    int[] c = isPrimary ? primaryColor : secondaryColor;
                    setLED(stripStart + i, c);
                }
            } else if (progress < 0.8) {
                // SCANNER EXPLOSION — two heads racing outward from center
                double phaseProgress = (progress - 0.5) / 0.3; // 0→1 within this phase
                double center = stripCount / 2.0;
                double reach = phaseProgress * (stripCount / 2.0); // How far from center
                int glowWidth = 4;
                
                // Background: dim pulsing alliance color
                double bgPulse = 0.1 + 0.15 * ((Math.sin(currentTime * 8.0) + 1) / 2);
                
                for (int i = 0; i < stripCount; i++) {
                    double distFromCenter = Math.abs(i - center);
                    double distFromHead = Math.abs(distFromCenter - reach);
                    
                    if (distFromHead < 1.0) {
                        // Scanner head — bright white
                        setLED(stripStart + i, secondaryColor);
                    } else if (distFromHead < glowWidth) {
                        // Glow tail
                        double falloff = 1.0 - (distFromHead / glowWidth);
                        setLED(stripStart + i, scaleColor(primaryColor, falloff * falloff));
                    } else if (distFromCenter < reach) {
                        // Filled area behind scanner — alliance color
                        setLED(stripStart + i, scaleColor(primaryColor, 0.6));
                    } else {
                        // Not yet reached — dim background
                        setLED(stripStart + i, scaleColor(primaryColor, bgPulse));
                    }
                }
                
                // Onboard: solid alliance color
                for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
                    ledBuffer[i][0] = (int)(primaryColor[0] * masterBrightness);
                    ledBuffer[i][1] = (int)(primaryColor[1] * masterBrightness);
                    ledBuffer[i][2] = (int)(primaryColor[2] * masterBrightness);
                }
            } else {
                // TRIUMPHANT SOLID — bright alliance color with slow power pulse
                double brightness = 0.8 + 0.2 * ((Math.sin(currentTime * 3.0) + 1) / 2);
                int[] c = scaleColor(primaryColor, brightness);
                
                for (int i = 0; i < ledCount; i++) {
                    ledBuffer[i][0] = (int)(c[0] * masterBrightness);
                    ledBuffer[i][1] = (int)(c[1] * masterBrightness);
                    ledBuffer[i][2] = (int)(c[2] * masterBrightness);
                }
            }
        } else {
            // === THEY GO FIRST — Acknowledge then prepare ===
            
            // Phase 1 (0-0.4): Their color rapid flash (acknowledging)
            // Phase 2 (0.4-1.0): Fade from their color to our color breathing
            
            if (progress < 0.4) {
                // Their color rapid strobe — 8Hz
                double strobeHz = 8.0;
                boolean isOn = ((int)(currentTime * strobeHz * 2) % 2) == 0;
                int[] c = isOn ? primaryColor : Constants.LEDs.OFF;
                for (int i = 0; i < ledCount; i++) {
                    ledBuffer[i][0] = (int)(c[0] * masterBrightness);
                    ledBuffer[i][1] = (int)(c[1] * masterBrightness);
                    ledBuffer[i][2] = (int)(c[2] * masterBrightness);
                }
            } else {
                // Crossfade from their color to ours with breathing
                double fadeProgress = (progress - 0.4) / 0.6; // 0→1
                int[] blended = lerpColor(primaryColor, secondaryColor, fadeProgress);
                double breath = 0.5 + 0.5 * ((Math.sin(currentTime * 2.0) + 1) / 2);
                int[] c = scaleColor(blended, breath);
                for (int i = 0; i < ledCount; i++) {
                    ledBuffer[i][0] = (int)(c[0] * masterBrightness);
                    ledBuffer[i][1] = (int)(c[1] * masterBrightness);
                    ledBuffer[i][2] = (int)(c[2] * masterBrightness);
                }
            }
        }
        
        pushBuffer();
    }
    
    /** Returns the other alliance's color. */
    private int[] getOtherAllianceColor() {
        // If we're blue, other is red, and vice versa
        if (allianceColor == Constants.LEDs.BLUE_ALLIANCE) {
            return Constants.LEDs.RED_ALLIANCE;
        }
        return Constants.LEDs.BLUE_ALLIANCE;
    }
    
    // =========================================================================
    // PATTERN: PROGRESSIVE CHASE — Dot(s) racing along the strip, speed ramps up
    // =========================================================================
    
    /**
     * A colored dot chases along the strip with a fading tail. As urgency
     * increases, the tail grows LONGER — eventually filling the entire strip
     * at max urgency (solid color). Speed also increases but stays readable.
     * 
     * @param color    RGB color of the chase dot
     * @param urgency  0.0 = relaxed (short tail, slow), 1.0 = maximum (full strip, fast).
     *                 Values outside [0,1] are clamped.
     */
    private void setProgressiveChase(int[] color, double urgency) {
        urgency = Math.max(0.0, Math.min(1.0, urgency));
        
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        
        // Speed: moderate ramp so the chase stays readable even at high urgency.
        // Slow (urgency 0) = ~0.4 strip/sec → Fast (urgency 1) = ~2 strip/sec
        double stripsPerSec = 0.4 + 1.6 * urgency;
        double ledsPerSec = stripsPerSec * stripCount;
        
        // Tail length GROWS with urgency — the strip "fills up" as time runs out.
        // urgency 0.0 → ~5 LEDs (small dot), urgency 1.0 → full strip (solid)
        int tailLen = (int)(5 + (stripCount - 5) * urgency * urgency);
        tailLen = Math.min(stripCount, Math.max(3, tailLen));
        
        // Head position: continuously advancing, wraps around
        double headPos = (Timer.getFPGATimestamp() * ledsPerSec) % stripCount;
        
        clearBuffer();
        
        // Onboard LEDs: brightness scales with urgency
        double onboardBright = 0.10 + 0.90 * urgency;
        int[] onboardColor = scaleColor(color, onboardBright);
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            ledBuffer[i][0] = (int)(onboardColor[0] * masterBrightness);
            ledBuffer[i][1] = (int)(onboardColor[1] * masterBrightness);
            ledBuffer[i][2] = (int)(onboardColor[2] * masterBrightness);
        }
        
        // Strip: chase dot with growing tail
        for (int i = 0; i < stripCount; i++) {
            // Distance behind the head (wrapping)
            double distBehind = headPos - i;
            if (distBehind < 0) distBehind += stripCount;
            
            double brightness;
            if (distBehind < 1.0) {
                brightness = 1.0; // Head pixel — full bright
            } else if (distBehind < tailLen) {
                // Tail: smooth fade from head (1.0) to tip (dim but visible)
                double t = 1.0 - ((distBehind - 1.0) / (tailLen - 1.0));
                // At high urgency the tail floor rises (less fade, more solid)
                double tailFloor = 0.3 * urgency * urgency;
                brightness = tailFloor + (1.0 - tailFloor) * t * t;
            } else {
                brightness = 0.0; // Dark
            }
            
            if (brightness > 0.01) {
                int[] c = scaleColor(color, brightness);
                int idx = stripStart + i;
                ledBuffer[idx][0] = (int)(c[0] * masterBrightness);
                ledBuffer[idx][1] = (int)(c[1] * masterBrightness);
                ledBuffer[idx][2] = (int)(c[2] * masterBrightness);
            }
        }
        
        pushBuffer();
    }
    
    // =========================================================================
    // MAIN PATTERN DISPATCH — priority-based, simple and clear
    // =========================================================================
    
    /**
     * Updates LED pattern based on current state and action.
     * 
     * Priority order (highest first):
     *  1.  Brownout (safety)
     *  2.  E-Stop (FMS/DS level — excessive pattern)
     *  3.  Game Data Announcement (who goes first — 3s one-shot)
     *  4.  Firing (active scoring feedback — white strobe)
     *  5.  Ready to Shoot (green = pull trigger)
     *  6.  Spooling (orange breathing = warming up)
     *  7.  Intaking (solid yellow-green)
     *  8.  Force Shoot (purple breathing)
     *  9.  Defensive Brake (pulsing amber/gold)
     *  10. Match phase patterns (boot, disabled, auto, teleop chase, endgame, victory)
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
        
        // === PRIORITY 2: REAL E-STOP (FMS/DS level) — EXCESSIVE dramatic pattern ===
        if (isEStopped) {
            currentPatternName = "E-STOP!!!";
            setEStopPattern();
            return;
        }
        
        // === PRIORITY 3: GAME DATA ANNOUNCEMENT — robot knows who goes first! ===
        if (announcementActive) {
            double elapsed = Timer.getFPGATimestamp() - announcementStartTime;
            if (elapsed < ANNOUNCEMENT_DURATION) {
                currentPatternName = announcementWeGoFirst ? "WE GO FIRST!!!" : "They go first...";
                setGameDataAnnouncement(elapsed, announcementWeGoFirst);
                return;
            }
            announcementActive = false; // Announcement finished
        }
        
        // === PRIORITY 4: FIRING — balls are actively being fed ===
        if (currentAction == ActionState.FIRING) {
            currentPatternName = "FIRING!";
            setStrobe(Constants.LEDs.WHITE, 15.0);
            return;
        }
        
        // === PRIORITY 5: READY TO SHOOT — solid GREEN ===
        // This is the critical "you can pull the trigger now" indicator.
        // Set by RobotContainer when shooter.isReady() during LT spool.
        if (currentAction == ActionState.SHOOTING) {
            currentPatternName = "READY - SHOOT!";
            setSolidColor(Constants.LEDs.SHOOTING_COLOR);
            return;
        }
        
        // === PRIORITY 6: SPOOLING — orange breathing ===
        if (currentAction == ActionState.SPOOLING) {
            currentPatternName = "Spooling Up";
            setBreathing(Constants.LEDs.SPOOLING_COLOR, 1.0);
            return;
        }
        
        // === PRIORITY 7: INTAKING — solid yellow-green ===
        if (currentAction == ActionState.INTAKING) {
            currentPatternName = "Intaking";
            setSolidColor(Constants.LEDs.INTAKING_COLOR);
            return;
        }
        
        // === PRIORITY 8: FORCE SHOOT — purple breathing ===
        if (currentAction == ActionState.FORCE_SHOOT) {
            currentPatternName = "Force Shoot";
            setBreathing(Constants.LEDs.AIMING_COLOR, 1.5);
            return;
        }
        
        // === PRIORITY 9: DEFENSIVE — pulsing amber/gold ===
        if (currentAction == ActionState.DEFENSIVE) {
            currentPatternName = "Defensive Brake";
            setBreathing(Constants.LEDs.TEAM_GOLD, 0.8);
            return;
        }
        
        // === PRIORITY 10: Match phase patterns ===
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
                // No FMS timing (practice mode) — show a gentle chase with no urgency ramp
                if (matchTime < 0) {
                    currentPatternName = "Teleop (Practice)";
                    setProgressiveChase(allianceColor, 0.15);
                    break;
                }
                
                // Endgame (last 30 seconds) — starts urgent, ends at max
                // Urgency 0.5→1.0 so the chase tail is already half-full entering endgame
                if (phase == GamePhase.END_GAME) {
                    double endgameUrgency = (matchTime > 0)
                        ? 0.5 + 0.5 * (1.0 - matchTime / 30.0)
                        : 1.0;
                    currentPatternName = String.format("ENDGAME (%.0fs, urg=%.2f)", matchTime, endgameUrgency);
                    setProgressiveChase(allianceColor, endgameUrgency);
                    break;
                }
                
                // --- Normal shift-based teleop ---
                // Two states: we're active (scoring), or we're waiting.
                // Transition period folds into wait time.
                
                if (gameState.isOurAllianceActive() && phase != GamePhase.TRANSITION) {
                    // OUR SHIFT — alliance color chase fills up as our time runs out
                    double timeLeft = gameState.getTimeRemainingActive(); // 25→0
                    double activeUrgency = 1.0 - Math.min(1.0, timeLeft / 25.0); // 0→1
                    
                    // Last shift before endgame: boost urgency so it flows into endgame
                    if (matchTime > 0 && matchTime <= 55.0) {
                        // We're in SHIFT_4 territory — make urgency floor higher
                        double endgameApproach = 1.0 - ((matchTime - 30.0) / 25.0);
                        activeUrgency = Math.max(activeUrgency, endgameApproach * 0.5);
                    }
                    
                    currentPatternName = String.format("Active (%.0fs left, urg=%.2f)", timeLeft, activeUrgency);
                    setProgressiveChase(allianceColor, activeUrgency);
                } else {
                    // WAITING for our shift (includes TRANSITION phase)
                    double secsUntil = gameState.getSecondsUntilOurNextShift();
                    
                    // During TRANSITION, secsUntil might be 0 (both considered active).
                    // Fold transition into wait time.
                    if (phase == GamePhase.TRANSITION) {
                        secsUntil = Math.max(secsUntil, matchTime - 130.0 + 5.0);
                        if (secsUntil < 0) secsUntil = 5.0;
                    }
                    
                    // Urgency based on actual wait time (max wait is ~25s for one shift)
                    double waitUrgency = 1.0 - Math.min(1.0, secsUntil / 25.0); // 0→1
                    
                    // Color: other alliance color when far, blends to ours in last 5s
                    int[] waitColor;
                    Alliance active = gameState.getCurrentlyActiveAlliance();
                    int[] otherColor = (active != null)
                        ? ((active == Alliance.Red) ? Constants.LEDs.RED_ALLIANCE : Constants.LEDs.BLUE_ALLIANCE)
                        : allianceColor;
                    if (secsUntil <= 5.0) {
                        double blend = 1.0 - (secsUntil / 5.0);
                        waitColor = lerpColor(otherColor, allianceColor, blend);
                    } else {
                        waitColor = otherColor;
                    }
                    
                    currentPatternName = String.format("Waiting (%.0fs, urg=%.2f)", secsUntil, waitUrgency);
                    setProgressiveChase(waitColor, waitUrgency);
                }
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
        
        // --- Check for game data announcement ---
        if (gameState.didJustReceiveGameMessage() && !announcementActive) {
            announcementActive = true;
            announcementStartTime = Timer.getFPGATimestamp();
            announcementWeGoFirst = gameState.doWeGoFirst();
            
            // Send Elastic notification
            String who = announcementWeGoFirst ? "WE GO FIRST!" : "They go first.";
            Alliance first = gameState.getFirstActiveAlliance();
            String detail = (first == Alliance.Blue ? "Blue" : "Red") + " alliance scores first.";
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(announcementWeGoFirst ? NotificationLevel.INFO : NotificationLevel.WARNING)
                .withTitle(who)
                .withDescription(detail)
                .withDisplaySeconds(5.0));
        }
        
        // --- Throttled pattern update (~10Hz) ---
        // Announcement runs at full 50Hz for crisp strobes
        ledUpdateCounter++;
        if (announcementActive || ledUpdateCounter >= LED_UPDATE_DIVISOR) {
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