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
 * Every pattern is per-LED addressable for maximum visual impact.
 * High contrast (vivid color on black), sharp edges, constant motion.
 *
 * PHYSICAL LAYOUT (back strip only — belly strip disconnected):
 *   Indices 0-7:   Onboard LEDs
 *   Indices 8-45:  Back/top strip (38 LEDs)
 *
 * PATTERN SUMMARY:
 *   DISABLED       = Lava flow — bright team-color blobs drift through black
 *   AUTO           = Twin comets — two bright bolts racing in opposite directions
 *   TELEOP ACTIVE  = Pulse wave — sharp bouncing glow, speeds up with urgency
 *   TELEOP WAITING = Other alliance color wave, blends to ours in last 5s
 *   ENDGAME        = Frantic pulse wave ramping to solid
 *   FIRING         = Lightning cascade — white fills sweep left-right rapidly
 *   READY TO SHOOT = Reactor core — green waves radiate outward from center
 *   SPOOLING       = Power charge — orange fill bar with hot leading edge
 *   INTAKING       = Vacuum pull — dots sweep inward from edges to center
 *   FORCE SHOOT    = Plasma field — purple interference waves shimmer
 *   DEFENSIVE      = Shield pulse — gold bars radiate outward from center
 *   NO AUTO        = Alarm sweep — red/orange bar sweeps back and forth
 *   BROWNOUT       = Dim amber flicker (safety)
 *   E-STOP         = Maximum chaos: triple scanner + strobe + flash-bangs
 *   GAME DATA      = Dramatic announcement with edge race or wipe
 *   GAME DATA LOST = White glitch scanner
 *   ZONE BLIP      = Alliance color ring radiates from center
 *   VICTORY        = Grand finale: bursts + rainbow + sparkle rain + strobe
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

    // LED update throttle: ~25Hz (every other 50Hz cycle)
    private int ledUpdateCounter = 0;
    private static final int LED_UPDATE_DIVISOR = 2;

    // Shift timing warning flash system
    private double warningFlashStart = -1;
    private int[] warningFlashColor = null;
    private String warningFlashName = null;
    private static final double WARNING_FLASH_DURATION = 0.75; // seconds

    // One-shot tracking for shift timing warnings (reset when alliance becomes active)
    private boolean hasFlashedShift10s = false;
    private boolean hasFlashedShift5s = false;
    private boolean hasFlashedShift1s = false;

    // One-shot tracking for endgame countdown warnings
    private boolean hasFlashedEndgame10s = false;
    private boolean hasFlashedEndgame5s = false;

    // Game data lost animation
    private boolean gameDataLostActive = false;

    // Zone entry blip system
    private double blipStartTime = -1;
    private int[] blipColor = null;
    private String blipName = null;
    private static final double BLIP_DURATION = 0.4; // seconds

    // =========================================================================
    // ENUMS
    // =========================================================================

    /**
     * LED states for robot operation phases.
     * TELEOP handles endgame internally via GamePhase — no separate ENDGAME state needed.
     */
    public enum LEDState {
        BOOT_WARMUP,    // Team colors aurora while CANdle warms up
        DISABLED,       // Lava flow (pits / pre-match)
        AUTO,           // Twin comets
        TELEOP,         // Pulse wave — speed/width/brightness vary by shift/endgame
        ENDGAME,        // (Unused — endgame handled inside TELEOP via GamePhase.END_GAME)
        MATCH_END,      // Grand finale celebration
        BROWNOUT        // Dim amber flicker
    }

    /**
     * Robot action states — each gets its own per-LED animation.
     * Higher-priority actions override lower ones in the pattern dispatch.
     */
    public enum ActionState {
        IDLE,        // No special action — show base state pattern
        FIRING,      // Lightning cascade — white fills sweep
        SHOOTING,    // Reactor core — green waves radiate from center
        SPOOLING,    // Power charge — orange fill bar
        INTAKING,    // Vacuum pull — dots sweep inward
        DEFENSIVE,   // Shield pulse — gold bars radiate outward
        FORCE_SHOOT, // Plasma field — purple interference pattern
        NO_AUTO,     // Alarm sweep — red/orange scanner
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

    /** Check if the CANdle is responding on the CAN bus. */
    public boolean checkHealth() {
        return candle.isConnected();
    }

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
        factor = Math.max(0.0, factor);
        // Gentle gamma (1.2) keeps colors vivid at medium brightness
        double gFactor = factor <= 1.0 ? Math.pow(factor, 1.2) : factor;
        return new int[] {
            Math.min(255, Math.max(0, (int)(color[0] * gFactor))),
            Math.min(255, Math.max(0, (int)(color[1] * gFactor))),
            Math.min(255, Math.max(0, (int)(color[2] * gFactor)))
        };
    }

    /** 1D pseudo-noise for organic animations. */
    private double noise1D(double x) {
        double v = Math.sin(x * 1.0) * 0.5
                 + Math.sin(x * 2.3 + 1.7) * 0.25
                 + Math.sin(x * 5.1 + 3.1) * 0.125;
        return (v / 0.875 + 1.0) / 2.0;
    }

    /** Converts a hue (0-1) to RGB. Used for rainbow effects. */
    private int[] hueToRGB(double hue) {
        hue = hue % 1.0;
        if (hue < 0) hue += 1.0;
        double r, g, b;
        if (hue < 1.0 / 6.0) {
            r = 1; g = hue * 6; b = 0;
        } else if (hue < 2.0 / 6.0) {
            r = 1 - (hue - 1.0 / 6.0) * 6; g = 1; b = 0;
        } else if (hue < 3.0 / 6.0) {
            r = 0; g = 1; b = (hue - 2.0 / 6.0) * 6;
        } else if (hue < 4.0 / 6.0) {
            r = 0; g = 1 - (hue - 3.0 / 6.0) * 6; b = 1;
        } else if (hue < 5.0 / 6.0) {
            r = (hue - 4.0 / 6.0) * 6; g = 0; b = 1;
        } else {
            r = 1; g = 0; b = 1 - (hue - 5.0 / 6.0) * 6;
        }
        return new int[]{(int)(r * 255), (int)(g * 255), (int)(b * 255)};
    }

    /** Smoothstep: sharp S-curve transition 0→1. */
    private double smoothstep(double x) {
        x = Math.max(0.0, Math.min(1.0, x));
        return x * x * (3 - 2 * x);
    }

    // =========================================================================
    // SIMPLE ANIMATION PRIMITIVES
    // =========================================================================

    /** Triangle wave: 0→1→0 over one period. Used for scanner bounce. */
    private double triangleWave(double t) {
        double phase = t % 1.0;
        if (phase < 0) phase += 1.0;
        return phase < 0.5 ? (phase * 2.0) : (1.0 - (phase - 0.5) * 2.0);
    }

    /**
     * Strobe — on/off flashing at given Hz.
     */
    private void setStrobe(int[] color, double hz) {
        boolean isOn = ((int)(Timer.getFPGATimestamp() * hz) % 2) == 0;
        setSolidColor(isOn ? color : Constants.LEDs.OFF);
    }

    // =========================================================================
    // PATTERN: DISABLED — Lava Flow
    // =========================================================================

    /**
     * Bright blobs of team orange and blue drift through darkness.
     * Sharp color zones with true black between them. Gold ember sparks
     * streak through at high speed. High contrast, vivid, organic.
     */
    private void setIdleEmberPattern() {
        double t = Timer.getFPGATimestamp();
        int[] orange = Constants.LEDs.TEAM_SAFETY_ORANGE;
        int[] blue = Constants.LEDs.TEAM_BLUE;
        int[] gold = Constants.LEDs.TEAM_GOLD;
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;

        clearBuffer();

        // Onboard: slow vivid crossfade
        double onboardPhase = smoothstep(Math.sin(t * 0.8) * 0.5 + 0.5);
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, lerpColor(orange, blue, onboardPhase));
        }

        for (int i = 0; i < stripCount; i++) {
            // Sharp color zones: double-smoothstep creates distinct orange/blue regions
            double rawBlend = Math.sin((i + t * 3.0) * 0.2) * 0.5 + 0.5;
            double blend = smoothstep(smoothstep(rawBlend));
            int[] baseColor = lerpColor(orange, blue, blend);

            // Brightness: two multiplied waves create bright peaks with dark valleys
            double wave1 = Math.sin((i + t * 3.0) * 0.3) * 0.5 + 0.5;
            double wave2 = Math.sin((i - t * 1.5) * 0.45 + 2.0) * 0.5 + 0.5;
            double brightness = wave1 * wave2;
            brightness = 0.03 + 0.97 * brightness;

            // Gold accent blooms
            double goldWave = Math.max(0, Math.sin((i + t * 2.5) * 0.4) - 0.8) / 0.2;
            if (goldWave > 0) {
                baseColor = lerpColor(baseColor, gold, goldWave * 0.6);
                brightness = Math.max(brightness, goldWave * 0.9);
            }

            // Fire flicker: random pixels pop to full brightness
            double flicker = noise1D(i * 4.3 + t * 12.0);
            if (flicker > 0.9) {
                double pop = (flicker - 0.9) / 0.1;
                brightness = Math.max(brightness, pop);
            }

            setLED(stripStart + i, scaleColor(baseColor, brightness));
        }

        pushBuffer();
    }

    // =========================================================================
    // PATTERN: AUTO — Twin Comets
    // =========================================================================

    /**
     * Two bright comets race in opposite directions. White-hot heads with
     * long alliance-color tails that fade to black with cubic falloff.
     * Pure black background for maximum contrast.
     */
    private void setAutoSurgePattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;

        double cycleTime = 0.8;
        double phase = (t % cycleTime) / cycleTime;

        double comet1Pos = phase * (stripCount - 1);
        double comet2Pos = (1.0 - phase) * (stripCount - 1);
        int tailLength = 12;

        clearBuffer();

        // Onboard: rapid alliance color pulse
        double onboardPulse = 0.4 + 0.6 * Math.abs(Math.sin(t * 8.0));
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, scaleColor(allianceColor, onboardPulse));
        }

        for (int i = 0; i < stripCount; i++) {
            double brightness = 0;
            boolean isHead = false;

            // Comet 1: moving right
            double dist1 = comet1Pos - i;
            if (dist1 >= -1.0 && dist1 < tailLength) {
                if (dist1 < 1.0) {
                    brightness = Math.max(brightness, 1.0 - Math.max(0, Math.abs(dist1)));
                    isHead = true;
                } else {
                    double tailFade = 1.0 - (dist1 / tailLength);
                    brightness = Math.max(brightness, tailFade * tailFade * tailFade);
                }
            }

            // Comet 2: moving left
            double dist2 = i - comet2Pos;
            if (dist2 >= -1.0 && dist2 < tailLength) {
                if (dist2 < 1.0) {
                    brightness = Math.max(brightness, 1.0 - Math.max(0, Math.abs(dist2)));
                    isHead = true;
                } else {
                    double tailFade = 1.0 - (dist2 / tailLength);
                    brightness = Math.max(brightness, tailFade * tailFade * tailFade);
                }
            }

            if (brightness > 0.01) {
                if (isHead && brightness > 0.5) {
                    setLED(stripStart + i, lerpColor(allianceColor, Constants.LEDs.WHITE, brightness));
                } else {
                    setLED(stripStart + i, scaleColor(allianceColor, brightness));
                }
            }
        }

        pushBuffer();
    }

    // =========================================================================
    // PATTERN: VICTORY — Grand Finale
    // =========================================================================

    /**
     * Four-phase cycling celebration:
     *   Phase A: Firework bursts — expanding rings from random positions
     *   Phase B: Rainbow wave — dual-direction flowing spectrum
     *   Phase C: Gold sparkle rain over vivid team color base
     *   Phase D: Rapid alliance/white/gold color strobe
     */
    private void setVictoryFireworks() {
        double t = Timer.getFPGATimestamp();
        double elapsed = t - stateStartTime;
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;

        int[] gold = Constants.LEDs.VICTORY_GOLD;
        int[] orange = Constants.LEDs.TEAM_SAFETY_ORANGE;
        int[] blue = Constants.LEDs.TEAM_BLUE;
        int[] white = Constants.LEDs.WHITE;

        double phaseDuration = 2.5;
        int phaseNum = ((int)(elapsed / phaseDuration)) % 4;

        clearBuffer();

        // Onboard: rapid color cycle
        int onboardCycle = ((int)(t * 12)) % 4;
        int[] onboardColor;
        switch (onboardCycle) {
            case 0: onboardColor = gold; break;
            case 1: onboardColor = white; break;
            case 2: onboardColor = orange; break;
            default: onboardColor = blue; break;
        }
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, onboardColor);
        }

        if (phaseNum == 0) {
            // === FIREWORK BURSTS — bigger, more frequent ===
            for (int i = 0; i < stripCount; i++) {
                double brightness = 0.0;
                int[] burstColor = gold;

                for (int b = 0; b < 5; b++) {
                    double burstCycle = t * 2.0 + b * 0.5;
                    double burstTime = burstCycle % 0.8;
                    double burstSeed = Math.floor(burstCycle / 0.8) * 5.0 + b * 17.3;
                    double burstCenter = noise1D(burstSeed) * stripCount;
                    double burstRadius = burstTime * (stripCount * 0.5);
                    double burstFade = 1.0 - (burstTime / 0.8);

                    double distFromBurst = Math.abs(i - burstCenter);
                    if (distFromBurst < burstRadius && distFromBurst > burstRadius - 5) {
                        double ringBright = burstFade * (1.0 - (burstRadius - distFromBurst) / 5.0);
                        if (ringBright > brightness) {
                            brightness = ringBright;
                            switch (b % 4) {
                                case 0: burstColor = gold; break;
                                case 1: burstColor = white; break;
                                case 2: burstColor = orange; break;
                                default: burstColor = blue; break;
                            }
                        }
                    }
                }

                double sparkle = noise1D(i * 2.1 + t * 15.0);
                if (sparkle > 0.83) {
                    double sparkBright = (sparkle - 0.83) / 0.17;
                    if (sparkBright > brightness) {
                        brightness = sparkBright;
                        burstColor = white;
                    }
                }

                brightness = Math.max(0.05, brightness);
                setLED(stripStart + i, scaleColor(burstColor, brightness));
            }
        } else if (phaseNum == 1) {
            // === RAINBOW WAVE — dual-direction crossing ===
            for (int i = 0; i < stripCount; i++) {
                double hue1 = ((double) i / stripCount + t * 0.8) % 1.0;
                double hue2 = ((double) i / stripCount - t * 0.5 + 0.5) % 1.0;
                int[] rgb1 = hueToRGB(hue1);
                int[] rgb2 = hueToRGB(hue2);
                int[] mixed = lerpColor(rgb1, rgb2, 0.5 + 0.3 * Math.sin(t * 3.0));
                setLED(stripStart + i, mixed);
            }
        } else if (phaseNum == 2) {
            // === GOLD SPARKLE RAIN — dense, vivid ===
            for (int i = 0; i < stripCount; i++) {
                double colorBlend = Math.sin(i * 0.3 + t * 0.5) * 0.5 + 0.5;
                int[] baseColor = lerpColor(orange, blue, colorBlend);
                double baseBright = 0.15;

                double rain1 = noise1D(i * 1.3 + t * 8.0);
                double rain2 = noise1D(i * 2.7 + t * 10.0 + 50);
                double rain3 = noise1D(i * 0.9 + t * 14.0 + 100);
                double maxRain = Math.max(rain1, Math.max(rain2, rain3));

                if (maxRain > 0.75) {
                    double sparkBright = (maxRain - 0.75) / 0.25;
                    int[] sparkColor = (maxRain == rain1) ? gold : ((maxRain == rain2) ? white : orange);
                    setLED(stripStart + i, lerpColor(scaleColor(baseColor, baseBright), sparkColor, sparkBright));
                } else {
                    setLED(stripStart + i, scaleColor(baseColor, baseBright));
                }
            }
        } else {
            // === RAPID COLOR STROBE — alliance, white, gold cycling ===
            int strobePhase = ((int)(t * 8.0)) % 3;
            int[] strobeColor;
            switch (strobePhase) {
                case 0: strobeColor = allianceColor; break;
                case 1: strobeColor = white; break;
                default: strobeColor = gold; break;
            }
            for (int i = 0; i < stripCount; i++) {
                setLED(stripStart + i, strobeColor);
            }
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
    // PATTERN: E-STOP — MAXIMUM ALERT
    // =========================================================================

    /**
     * Triple red scanner sweep, 12Hz strobe overlay, random white flash-bangs,
     * aggressive pulsing background, 16Hz alternating chaos on onboard LEDs.
     */
    private void setEStopPattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int[] red = Constants.LEDs.ESTOP_COLOR;
        int[] white = Constants.LEDs.WHITE;
        int[] hotCore = Constants.LEDs.ESTOP_CORE;

        boolean strobeOn = ((int)(t * 12.0) % 2) == 0;
        boolean flashBang = noise1D(t * 47.0) > 0.88;
        double bgPulse = 0.12 + 0.30 * ((Math.sin(t * 6.0) + 1) / 2);
        int[] dimRed = scaleColor(red, bgPulse);

        double scanPos1 = triangleWave(t * 2.5);
        double scanPos2 = triangleWave(t * 2.5 + 0.33);
        double scanPos3 = triangleWave(t * 3.7 + 0.15);
        double head1 = scanPos1 * (stripCount - 1);
        double head2 = scanPos2 * (stripCount - 1);
        double head3 = scanPos3 * (stripCount - 1);
        int scanWidth = Math.max(5, stripCount / 5);

        clearBuffer();

        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            boolean thisOn = ((int)(t * 16.0 + i * 3.7) % 2) == 0;
            int[] c = thisOn ? red : (flashBang ? white : hotCore);
            ledBuffer[i][0] = (int)(c[0] * masterBrightness);
            ledBuffer[i][1] = (int)(c[1] * masterBrightness);
            ledBuffer[i][2] = (int)(c[2] * masterBrightness);
        }

        if (flashBang) {
            for (int i = 0; i < stripCount; i++) {
                setLED(stripStart + i, white);
            }
        } else {
            for (int i = 0; i < stripCount; i++) {
                double dist1 = Math.abs(i - head1);
                double dist2 = Math.abs(i - head2);
                double dist3 = Math.abs(i - head3);
                double minDist = Math.min(dist1, Math.min(dist2, dist3));

                double brightness;
                if (minDist <= 1.0) {
                    brightness = 1.0;
                } else if (minDist <= scanWidth) {
                    double falloff = 1.0 - (minDist / scanWidth);
                    brightness = falloff * falloff;
                } else {
                    brightness = 0.0;
                }

                if (brightness > 0.5) {
                    int[] headColor = strobeOn ? white : hotCore;
                    setLED(stripStart + i, scaleColor(headColor, brightness));
                } else if (brightness > 0.01) {
                    setLED(stripStart + i, scaleColor(red, brightness));
                } else {
                    if (strobeOn) {
                        setLED(stripStart + i, scaleColor(red, bgPulse * 1.5));
                    } else {
                        setLED(stripStart + i, dimRed);
                    }
                }
            }
        }

        pushBuffer();
    }

    // =========================================================================
    // PATTERN: GAME DATA LOST — Glitch Scanner
    // =========================================================================

    /**
     * White scanning bar sweeps over black with periodic glitch bursts —
     * random pixels flash white for a frame, creating a "signal lost" look.
     */
    private void setGameDataLostPattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;

        double scanPos = triangleWave(t * 0.67);
        double headPos = scanPos * (stripCount - 1);
        int scanWidth = 5;

        // Periodic glitch: every ~0.5s, random pixels flash for ~0.08s
        double glitchCycle = t * 2.0;
        boolean inGlitch = (glitchCycle % 1.0) < 0.16;

        clearBuffer();

        double onboardBreath = 0.1 + 0.2 * Math.sin(t * 2.0);
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            if (inGlitch && noise1D(i * 7.0 + t * 50.0) > 0.6) {
                setLED(i, Constants.LEDs.WHITE);
            } else {
                setLED(i, scaleColor(Constants.LEDs.WHITE, onboardBreath));
            }
        }

        for (int i = 0; i < stripCount; i++) {
            // Glitch: random pixels flash white
            if (inGlitch && noise1D(i * 3.1 + t * 50.0) > 0.7) {
                double glitchBright = noise1D(i * 5.7 + t * 80.0);
                setLED(stripStart + i, scaleColor(Constants.LEDs.WHITE, glitchBright));
                continue;
            }

            double dist = Math.abs(i - headPos);
            if (dist < 1.0) {
                setLED(stripStart + i, Constants.LEDs.WHITE);
            } else if (dist < scanWidth) {
                double falloff = 1.0 - ((dist - 1.0) / (scanWidth - 1.0));
                setLED(stripStart + i, scaleColor(Constants.LEDs.WHITE, falloff * falloff));
            }
        }

        pushBuffer();
    }

    // =========================================================================
    // PATTERN: ZONE ENTRY BLIP — Ring radiates from center
    // =========================================================================

    /**
     * Alliance-color ring expands outward from center and fades.
     * Much more dramatic than a flat flash — reads as "event happened HERE."
     */
    private void setZoneBlipPattern(int[] color, double elapsed) {
        double progress = elapsed / BLIP_DURATION;
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;

        // Ring expands outward from center
        double ringRadius = progress * (stripCount / 2.0 + 4);
        double ringWidth = 4.0;
        // Fade out over time
        double fadeOut = Math.pow(1.0 - progress, 1.5);

        clearBuffer();

        // Onboard: flash and fade
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, scaleColor(color, fadeOut));
        }

        for (int i = 0; i < stripCount; i++) {
            double dist = Math.abs(i - center);
            double ringDist = Math.abs(dist - ringRadius);

            if (ringDist < ringWidth) {
                double ringBright = (1.0 - ringDist / ringWidth) * fadeOut;
                // Leading edge is brightest
                if (dist > ringRadius - 1.5 && dist < ringRadius + 1.5) {
                    setLED(stripStart + i, lerpColor(color, Constants.LEDs.WHITE, ringBright * 0.4));
                } else {
                    setLED(stripStart + i, scaleColor(color, ringBright));
                }
            }
        }

        pushBuffer();
    }

    private void triggerBlip(String name, int[] color) {
        blipStartTime = Timer.getFPGATimestamp();
        blipColor = color;
        blipName = name;
    }

    // =========================================================================
    // PATTERN: GAME DATA ANNOUNCEMENT
    // =========================================================================

    /**
     * WE GO FIRST: Edge race build-up + explosion + triumphant solid.
     * THEY GO FIRST: Their color wipe + our color overwipe + confident pulse.
     */
    private void setGameDataAnnouncement(double elapsed, boolean weGoFirst) {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;

        double progress = Math.min(1.0, elapsed / ANNOUNCEMENT_DURATION);

        int[] primaryColor = weGoFirst ? allianceColor : getOtherAllianceColor();
        int[] secondaryColor = weGoFirst ? Constants.LEDs.WHITE : allianceColor;

        clearBuffer();

        if (weGoFirst) {
            if (progress < 0.33) {
                double raceProgress = progress / 0.33;
                int litCount = (int)(raceProgress * (stripCount / 2 + 1));

                for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
                    setLED(i, scaleColor(primaryColor, 0.3 + 0.7 * raceProgress));
                }

                for (int i = 0; i < stripCount; i++) {
                    int distFromLeft = i;
                    int distFromRight = stripCount - 1 - i;
                    int minEdgeDist = Math.min(distFromLeft, distFromRight);

                    if (minEdgeDist < litCount) {
                        double edgeDist = litCount - minEdgeDist;
                        if (edgeDist <= 2) {
                            setLED(stripStart + i, lerpColor(primaryColor, Constants.LEDs.WHITE, 0.7));
                        } else {
                            double brightness = 0.5 + 0.3 * Math.sin(t * 8.0 + i);
                            setLED(stripStart + i, scaleColor(primaryColor, brightness));
                        }
                    } else {
                        setLED(stripStart + i, scaleColor(primaryColor, 0.03));
                    }
                }
            } else if (progress < 0.67) {
                boolean showPrimary = ((int)(t * 24) % 2) == 0;

                for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
                    setLED(i, ((i % 2 == 0) == showPrimary) ? primaryColor : secondaryColor);
                }

                int blockSize = 3;
                int shift = (int)(t * 30) % (blockSize * 2);
                for (int i = 0; i < stripCount; i++) {
                    boolean isPrimary = ((i + shift) / blockSize) % 2 == 0;
                    setLED(stripStart + i, isPrimary ? primaryColor : secondaryColor);
                }
            } else {
                double brightness = 0.8 + 0.2 * ((Math.sin(t * 3.0) + 1) / 2);
                int[] c = scaleColor(primaryColor, brightness);
                for (int i = 0; i < ledCount; i++) {
                    ledBuffer[i][0] = (int)(c[0] * masterBrightness);
                    ledBuffer[i][1] = (int)(c[1] * masterBrightness);
                    ledBuffer[i][2] = (int)(c[2] * masterBrightness);
                }
            }
        } else {
            if (progress < 0.27) {
                double wipeProgress = progress / 0.27;
                int wipeEdge = (int)(wipeProgress * stripCount);

                for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
                    setLED(i, scaleColor(primaryColor, 0.7));
                }

                for (int i = 0; i < stripCount; i++) {
                    if (i <= wipeEdge) {
                        double edgeDist = wipeEdge - i;
                        if (edgeDist < 3) {
                            setLED(stripStart + i, lerpColor(primaryColor, Constants.LEDs.WHITE, 0.4));
                        } else {
                            setLED(stripStart + i, scaleColor(primaryColor, 0.7));
                        }
                    }
                }
            } else if (progress < 0.6) {
                double wipeProgress = (progress - 0.27) / 0.33;
                int center = stripCount / 2;
                int reach = (int)(wipeProgress * center);

                for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
                    setLED(i, lerpColor(primaryColor, secondaryColor, wipeProgress));
                }

                for (int i = 0; i < stripCount; i++) {
                    int distFromCenter = Math.abs(i - center);
                    if (distFromCenter <= reach) {
                        if (distFromCenter >= reach - 2) {
                            setLED(stripStart + i, lerpColor(secondaryColor, Constants.LEDs.WHITE, 0.3));
                        } else {
                            setLED(stripStart + i, secondaryColor);
                        }
                    } else {
                        double fade = 0.7 * (1.0 - wipeProgress * 0.5);
                        setLED(stripStart + i, scaleColor(primaryColor, fade));
                    }
                }
            } else {
                double breath = 0.5 + 0.5 * ((Math.sin(t * 2.0) + 1) / 2);
                int[] c = scaleColor(secondaryColor, breath);
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
        if (allianceColor == Constants.LEDs.BLUE_ALLIANCE) {
            return Constants.LEDs.RED_ALLIANCE;
        }
        return Constants.LEDs.BLUE_ALLIANCE;
    }

    // =========================================================================
    // PATTERN: PULSE WAVE — Bouncing glow that ramps with urgency
    // =========================================================================

    /**
     * Sharp bouncing glow over dark background. As urgency increases:
     *   - Wave gets WIDER (4 → 20+ LEDs)
     *   - Wave moves FASTER (0.5 → 3 Hz)
     *   - Background brightness RISES toward solid
     *   - At urgency > 0.6: second wave appears
     *   - At urgency > 0.85: rapid strobe overlay
     * Cubic falloff for crisp edges. Pure color, no white tinting.
     */
    private void setPulseWave(int[] color, double urgency) {
        urgency = Math.max(0.0, Math.min(1.0, urgency));

        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        double t = Timer.getFPGATimestamp();

        double wavesPerSec = 0.5 + 2.5 * urgency;
        double waveHalfWidth = 2.0 + 14.0 * urgency;
        double baseBright = 0.02 + 0.35 * urgency * urgency;
        double peakBright = 0.85 + 0.15 * urgency;

        double headLED = triangleWave(t * wavesPerSec) * (stripCount - 1);

        boolean dualWave = urgency > 0.6;
        double head2LED = 0;
        if (dualWave) {
            head2LED = triangleWave(t * wavesPerSec * 0.8 + 0.5) * (stripCount - 1);
        }

        // Strobe overlay at high urgency
        double rapidPulse = 1.0;
        if (urgency > 0.85) {
            double pulseRate = 5.0 + 12.0 * ((urgency - 0.85) / 0.15);
            rapidPulse = 0.6 + 0.4 * ((Math.sin(t * pulseRate * 2 * Math.PI) + 1) / 2);
        }

        clearBuffer();

        double onboardBright = (baseBright + peakBright) * 0.5 * rapidPulse;
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, scaleColor(color, onboardBright));
        }

        for (int i = 0; i < stripCount; i++) {
            double dist1 = Math.abs(i - headLED);
            double waveBright1 = Math.max(0, 1.0 - dist1 / waveHalfWidth);
            waveBright1 = waveBright1 * waveBright1 * waveBright1; // Cubic falloff

            double waveBright2 = 0;
            if (dualWave) {
                double dist2 = Math.abs(i - head2LED);
                waveBright2 = Math.max(0, 1.0 - dist2 / waveHalfWidth);
                waveBright2 = waveBright2 * waveBright2 * waveBright2;
            }

            double totalWave = Math.min(1.0, waveBright1 + waveBright2);
            double brightness = baseBright + (peakBright - baseBright) * totalWave;
            brightness *= rapidPulse;

            setLED(stripStart + i, scaleColor(color, brightness));
        }

        pushBuffer();
    }

    // =========================================================================
    // ACTION STATE PATTERNS — Per-LED animations for every robot action
    // =========================================================================

    /**
     * FIRING: Lightning cascade — white fills sweep left-to-right then
     * right-to-left rapidly. Looks like electrical discharge.
     */
    private void setFiringPattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;

        double cycleTime = 0.15;
        double fullCycle = cycleTime * 2;
        double phase = (t % fullCycle);
        boolean leftToRight = phase < cycleTime;
        double fillProgress = (leftToRight ? phase : (phase - cycleTime)) / cycleTime;

        clearBuffer();

        boolean onboardOn = ((int)(t * 15.0) % 2) == 0;
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, onboardOn ? Constants.LEDs.WHITE : Constants.LEDs.OFF);
        }

        for (int i = 0; i < stripCount; i++) {
            double normalPos = leftToRight ? ((double) i / stripCount) : (1.0 - (double) i / stripCount);
            if (normalPos <= fillProgress) {
                double edgeDist = fillProgress - normalPos;
                double brightness = (edgeDist < 0.05) ? 1.0 : Math.max(0.25, 1.0 - edgeDist * 3.0);
                setLED(stripStart + i, scaleColor(Constants.LEDs.WHITE, brightness));
            }
        }

        pushBuffer();
    }

    /**
     * SHOOTING READY: Reactor core — bright green waves radiate outward
     * from center. Center always bright, waves expand outward continuously.
     */
    private void setShootingReadyPattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;

        double waveSpeed = 18.0;
        double waveSpacing = 9.0;

        clearBuffer();

        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, Constants.LEDs.SHOOTING_COLOR);
        }

        for (int i = 0; i < stripCount; i++) {
            double dist = Math.abs(i - center);
            double wavePhase = (dist - t * waveSpeed) % waveSpacing;
            if (wavePhase < 0) wavePhase += waveSpacing;
            double wave = Math.max(0, 1.0 - wavePhase / 3.0);
            wave = wave * wave;

            double distFade = 1.0 - (dist / (stripCount / 2.0)) * 0.4;
            double brightness = 0.25 + 0.75 * wave * distFade;

            setLED(stripStart + i, scaleColor(Constants.LEDs.SHOOTING_COLOR, brightness));
        }

        pushBuffer();
    }

    /**
     * SPOOLING: Power charge bar — orange fills from one end with a hot
     * leading edge, then resets. Visual "charging up" metaphor.
     */
    private void setSpoolingPattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;

        double cycleTime = 1.2;
        double phase = (t % cycleTime) / cycleTime;
        double fillEdge = phase * (stripCount + 3);

        clearBuffer();

        double onboardBreath = 0.2 + 0.8 * phase;
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, scaleColor(Constants.LEDs.SPOOLING_COLOR, onboardBreath));
        }

        for (int i = 0; i < stripCount; i++) {
            double distFromEdge = fillEdge - i;
            if (distFromEdge > 3) {
                // Filled: solid vivid orange
                setLED(stripStart + i, Constants.LEDs.SPOOLING_COLOR);
            } else if (distFromEdge > 0) {
                // Leading edge: hot white-orange glow
                double edgeBright = distFromEdge / 3.0;
                setLED(stripStart + i, lerpColor(Constants.LEDs.SPOOLING_COLOR,
                    Constants.LEDs.SPOOLING_HIGHLIGHT, edgeBright));
            } else if (distFromEdge > -2) {
                // Just ahead of edge: dim glow
                double aheadBright = 0.15 * (1.0 + distFromEdge / 2.0);
                setLED(stripStart + i, scaleColor(Constants.LEDs.SPOOLING_COLOR, aheadBright));
            } else {
                // Unfilled: near-black
                setLED(stripStart + i, scaleColor(Constants.LEDs.SPOOLING_COLOR, 0.03));
            }
        }

        pushBuffer();
    }

    /**
     * INTAKING: Vacuum pull — dots sweep inward from both edges toward
     * center. Gives a visual sense of "sucking in."
     */
    private void setIntakingPattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;

        double speed = 22.0;
        double dotSpacing = 8.0;

        clearBuffer();

        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, Constants.LEDs.INTAKING_COLOR);
        }

        for (int i = 0; i < stripCount; i++) {
            double distFromCenter = Math.abs(i - center);
            double maxDist = stripCount / 2.0;

            // Dots flow inward: as t increases, dots at each distance move toward center
            double flowPhase = (distFromCenter + t * speed) % dotSpacing;
            double dotBrightness = Math.max(0, 1.0 - flowPhase / 2.5);
            dotBrightness = dotBrightness * dotBrightness;

            // Brighter near center (where the intake draws things)
            double distFade = 0.3 + 0.7 * (1.0 - distFromCenter / maxDist);

            double brightness = 0.03 + dotBrightness * distFade;
            setLED(stripStart + i, scaleColor(Constants.LEDs.INTAKING_COLOR, brightness));
        }

        pushBuffer();
    }

    /**
     * FORCE SHOOT: Plasma field — two interfering sine waves create a
     * shimmering purple pattern with bright peaks and dark valleys.
     */
    private void setForceShootPattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;

        clearBuffer();

        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            double onPulse = 0.4 + 0.6 * Math.abs(Math.sin(t * 3.0 + i));
            setLED(i, scaleColor(Constants.LEDs.AIMING_COLOR, onPulse));
        }

        for (int i = 0; i < stripCount; i++) {
            double wave1 = Math.sin(i * 0.5 + t * 3.0);
            double wave2 = Math.sin(i * 0.7 - t * 2.0 + 1.5);
            double interference = (wave1 + wave2) * 0.5; // -1 to 1
            double brightness = 0.05 + 0.95 * Math.max(0, interference);

            setLED(stripStart + i, scaleColor(Constants.LEDs.AIMING_COLOR, brightness));
        }

        pushBuffer();
    }

    /**
     * DEFENSIVE: Shield pulse — gold bars radiate outward from center
     * like expanding radar rings. Hard edges, dark gaps.
     */
    private void setDefensivePattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int center = stripCount / 2;

        double speed = 14.0;
        double barWidth = 3.0;
        double barSpacing = 8.0;

        clearBuffer();

        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            double pulse = 0.4 + 0.6 * Math.abs(Math.sin(t * 4.0));
            setLED(i, scaleColor(Constants.LEDs.TEAM_GOLD, pulse));
        }

        for (int i = 0; i < stripCount; i++) {
            double dist = Math.abs(i - center);
            double barPhase = (dist - t * speed) % barSpacing;
            if (barPhase < 0) barPhase += barSpacing;

            double brightness;
            if (barPhase < barWidth) {
                brightness = 1.0 - barPhase / barWidth;
                brightness *= brightness;
            } else {
                brightness = 0.03;
            }

            // Fade at strip edges
            double edgeFade = Math.min(1.0, (stripCount / 2.0 - dist) / 3.0);
            edgeFade = Math.max(0, edgeFade);
            brightness *= edgeFade;

            setLED(stripStart + i, scaleColor(Constants.LEDs.TEAM_GOLD, brightness));
        }

        pushBuffer();
    }

    /**
     * NO AUTO: Alarm sweep — wide red/orange bar sweeps back and forth
     * over dim red background. Aggressive and alarming.
     */
    private void setNoAutoPattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;

        double scanPos = triangleWave(t * 3.0) * (stripCount - 1);
        int scanWidth = 8;

        clearBuffer();

        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            boolean isRed = ((int)(t * 4.0 + i) % 2) == 0;
            setLED(i, isRed ? Constants.LEDs.ESTOP_COLOR : Constants.LEDs.NO_AUTO_COLOR);
        }

        for (int i = 0; i < stripCount; i++) {
            double dist = Math.abs(i - scanPos);
            if (dist < scanWidth) {
                double brightness = 1.0 - dist / scanWidth;
                brightness *= brightness;
                int[] color = ((i / 2) % 2 == 0)
                    ? Constants.LEDs.ESTOP_COLOR : Constants.LEDs.NO_AUTO_COLOR;
                setLED(stripStart + i, scaleColor(color, brightness));
            } else {
                setLED(stripStart + i, scaleColor(Constants.LEDs.ESTOP_COLOR, 0.04));
            }
        }

        pushBuffer();
    }

    // =========================================================================
    // MAIN PATTERN DISPATCH — priority-based
    // =========================================================================

    /**
     * Updates LED pattern based on current state and action.
     *
     * Priority order (highest first):
     *  1.  Brownout (safety)
     *  2.  E-Stop (maximum alert)
     *  2.3 Game Data Lost (white glitch scanner)
     *  2.4 Zone Entry Blip (center radiate)
     *  2.5 Shift/Endgame Warning Flash
     *  3.  Game Data Announcement (who goes first)
     *  4.  Firing (lightning cascade)
     *  5.  Ready to Shoot (reactor core)
     *  6.  Spooling (power charge bar)
     *  7.  Intaking (vacuum pull)
     *  8.  Force Shoot (plasma field)
     *  9.  Defensive (shield pulse)
     *  10. No Auto (alarm sweep)
     *  11. Match phase patterns
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

        // === PRIORITY 2: REAL E-STOP ===
        if (isEStopped) {
            currentPatternName = "E-STOP!!!";
            setEStopPattern();
            return;
        }

        // === PRIORITY 2.3: GAME DATA LOST ===
        if (gameDataLostActive) {
            currentPatternName = "GAME DATA LOST!";
            setGameDataLostPattern();
            return;
        }

        // === PRIORITY 2.4: ZONE ENTRY BLIP ===
        if (blipColor != null) {
            double blipElapsed = Timer.getFPGATimestamp() - blipStartTime;
            if (blipElapsed < BLIP_DURATION) {
                currentPatternName = blipName;
                setZoneBlipPattern(blipColor, blipElapsed);
                return;
            }
            blipColor = null;
        }

        // === PRIORITY 2.5: SHIFT/ENDGAME WARNING FLASH ===
        if (warningFlashColor != null) {
            double elapsed = Timer.getFPGATimestamp() - warningFlashStart;
            if (elapsed < WARNING_FLASH_DURATION) {
                currentPatternName = warningFlashName;
                setStrobe(warningFlashColor, 10.0);
                return;
            }
            warningFlashColor = null;
        }

        // === PRIORITY 3: GAME DATA ANNOUNCEMENT ===
        if (announcementActive) {
            double elapsed = Timer.getFPGATimestamp() - announcementStartTime;
            if (elapsed < ANNOUNCEMENT_DURATION) {
                currentPatternName = announcementWeGoFirst ? "WE GO FIRST!!!" : "They go first...";
                setGameDataAnnouncement(elapsed, announcementWeGoFirst);
                return;
            }
            announcementActive = false;
        }

        // === PRIORITY 4: FIRING — lightning cascade ===
        if (currentAction == ActionState.FIRING) {
            currentPatternName = "FIRING!";
            setFiringPattern();
            return;
        }

        // === PRIORITY 5: READY TO SHOOT — reactor core ===
        if (currentAction == ActionState.SHOOTING) {
            currentPatternName = "READY - SHOOT!";
            setShootingReadyPattern();
            return;
        }

        // === PRIORITY 6: SPOOLING — power charge ===
        if (currentAction == ActionState.SPOOLING) {
            currentPatternName = "Spooling Up";
            setSpoolingPattern();
            return;
        }

        // === PRIORITY 7: INTAKING — vacuum pull ===
        if (currentAction == ActionState.INTAKING) {
            currentPatternName = "Intaking";
            setIntakingPattern();
            return;
        }

        // === PRIORITY 8: FORCE SHOOT — plasma field ===
        if (currentAction == ActionState.FORCE_SHOOT) {
            currentPatternName = "Force Shoot";
            setForceShootPattern();
            return;
        }

        // === PRIORITY 9: DEFENSIVE — shield pulse ===
        if (currentAction == ActionState.DEFENSIVE) {
            currentPatternName = "Defensive Brake";
            setDefensivePattern();
            return;
        }

        // === PRIORITY 10: NO AUTO — alarm sweep ===
        if (currentAction == ActionState.NO_AUTO) {
            currentPatternName = "NO AUTO!!!";
            setNoAutoPattern();
            return;
        }

        // === PRIORITY 11: Match phase patterns ===
        switch (currentState) {
            case BOOT_WARMUP:
                currentPatternName = "Boot Warmup";
                setIdleEmberPattern();
                if (timeSinceStateStart > Constants.LEDs.BOOT_ANIMATION_DURATION) {
                    setState(LEDState.DISABLED);
                }
                break;

            case DISABLED:
                currentPatternName = "Disabled (Lava Flow)";
                setIdleEmberPattern();
                break;

            case AUTO:
                currentPatternName = "Auto (Twin Comets)";
                setAutoSurgePattern();
                break;

            case TELEOP:
                // No FMS timing (practice mode) — gentle wave
                if (matchTime < 0) {
                    currentPatternName = "Teleop (Practice)";
                    setPulseWave(allianceColor, 0.15);
                    break;
                }

                // Endgame (last 30 seconds)
                if (phase == GamePhase.END_GAME) {
                    double endgameUrgency = (matchTime > 0)
                        ? 0.5 + 0.5 * (1.0 - matchTime / 30.0)
                        : 1.0;
                    currentPatternName = String.format("ENDGAME (%.0fs, urg=%.2f)", matchTime, endgameUrgency);
                    setPulseWave(allianceColor, endgameUrgency);
                    break;
                }

                // Normal shift-based teleop
                if (gameState.isOurAllianceActive() && phase != GamePhase.TRANSITION) {
                    double timeLeft = gameState.getTimeRemainingActive();
                    double activeUrgency = 1.0 - Math.min(1.0, timeLeft / 25.0);

                    if (matchTime > 0 && matchTime <= 55.0) {
                        double endgameApproach = 1.0 - ((matchTime - 30.0) / 25.0);
                        activeUrgency = Math.max(activeUrgency, endgameApproach * 0.5);
                    }

                    currentPatternName = String.format("Active (%.0fs left, urg=%.2f)", timeLeft, activeUrgency);
                    setPulseWave(allianceColor, activeUrgency);
                } else {
                    double secsUntil = gameState.getSecondsUntilOurNextShift();

                    if (phase == GamePhase.TRANSITION) {
                        secsUntil = Math.max(secsUntil, matchTime - 130.0 + 5.0);
                        if (secsUntil < 0) secsUntil = 5.0;
                    }

                    double waitUrgency = 1.0 - Math.min(1.0, secsUntil / 25.0);

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
                    setPulseWave(waitColor, waitUrgency);
                }
                break;

            case ENDGAME:
                currentPatternName = "ENDGAME!";
                setPulseWave(allianceColor, 0.85);
                break;

            case MATCH_END:
                currentPatternName = "Grand Finale!";
                setVictoryFireworks();
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
    // SHIFT & ENDGAME WARNING FLASH SYSTEM
    // =========================================================================

    /** Triggers a brief high-priority LED flash with an Elastic notification. */
    private void triggerWarningFlash(String name, int[] color, String notification) {
        warningFlashStart = Timer.getFPGATimestamp();
        warningFlashColor = color;
        warningFlashName = name;
        if (notification != null) {
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle(notification)
                .withDisplaySeconds(1.5));
        }
    }

    /**
     * Checks for shift timing warnings (10s/5s/1s before our shift) and
     * endgame countdown warnings (10s/5s remaining). Fires one-shot LED
     * flashes that override all action states so the driver never misses them.
     */
    private void checkShiftAndEndgameWarnings() {
        if (!DriverStation.isTeleop()) {
            hasFlashedShift10s = false;
            hasFlashedShift5s = false;
            hasFlashedShift1s = false;
            hasFlashedEndgame10s = false;
            hasFlashedEndgame5s = false;
            return;
        }

        GamePhase phase = gameState.getGamePhase();

        // --- Shift timing warnings ---
        if (!gameState.isOurAllianceActive() && phase != GamePhase.END_GAME
                && phase != GamePhase.TRANSITION) {
            double secsUntil = gameState.getSecondsUntilOurNextShift();

            if (secsUntil <= 10.0 && secsUntil > 9.0 && !hasFlashedShift10s) {
                hasFlashedShift10s = true;
                triggerWarningFlash("HEAD BACK 10s",
                        Constants.LEDs.ENDGAME_WARNING_10SEC, "HEAD BACK! 10 seconds");
            }
            if (secsUntil <= 5.0 && secsUntil > 4.0 && !hasFlashedShift5s) {
                hasFlashedShift5s = true;
                triggerWarningFlash("SPOOL UP 5s",
                        Constants.LEDs.ENDGAME_WARNING_5SEC, "START SPOOL! 5 seconds");
            }
            if (secsUntil <= 1.0 && secsUntil > 0.0 && !hasFlashedShift1s) {
                hasFlashedShift1s = true;
                triggerWarningFlash("SHOOT NOW 1s",
                        Constants.LEDs.SHOOTING_COLOR, "SHOOT NOW! 1 second");
            }
        } else {
            hasFlashedShift10s = false;
            hasFlashedShift5s = false;
            hasFlashedShift1s = false;
        }

        // --- Endgame countdown warnings ---
        if (phase == GamePhase.END_GAME) {
            double matchTime = DriverStation.getMatchTime();
            if (matchTime <= 10.0 && matchTime > 9.0 && !hasFlashedEndgame10s) {
                hasFlashedEndgame10s = true;
                triggerWarningFlash("ENDGAME 10s!",
                        Constants.LEDs.ENDGAME_WARNING_10SEC, "10 SECONDS LEFT!");
            }
            if (matchTime <= 5.0 && matchTime > 4.0 && !hasFlashedEndgame5s) {
                hasFlashedEndgame5s = true;
                triggerWarningFlash("ENDGAME 5s!",
                        Constants.LEDs.ENDGAME_WARNING_5SEC, "5 SECONDS LEFT!");
            }
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

        // --- Check shift timing and endgame warnings ---
        checkShiftAndEndgameWarnings();

        // --- Check for game data announcement ---
        if (gameState.didJustReceiveGameMessage() && !announcementActive) {
            announcementActive = true;
            announcementStartTime = Timer.getFPGATimestamp();
            announcementWeGoFirst = gameState.doWeGoFirst();

            String who = announcementWeGoFirst ? "WE GO FIRST!" : "They go first.";
            Alliance first = gameState.getFirstActiveAlliance();
            String detail = (first == Alliance.Blue ? "Blue" : "Red") + " alliance scores first.";
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(announcementWeGoFirst ? NotificationLevel.INFO : NotificationLevel.WARNING)
                .withTitle(who)
                .withDescription(detail)
                .withDisplaySeconds(5.0));
        }

        // --- Check for game data LOST ---
        if (gameState.didJustLoseGameData()) {
            gameDataLostActive = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.ERROR)
                .withTitle("GAME DATA LOST!")
                .withDescription("Game message went empty — check FMS connection")
                .withDisplaySeconds(5.0));
        }
        if (gameDataLostActive && !gameState.isGameDataLost()) {
            gameDataLostActive = false;
        }

        // --- Check for zone entry / shift activation blips ---
        if (gameState.didJustBecomeActive() && currentState == LEDState.TELEOP) {
            triggerBlip("SHIFT START!", allianceColor);
        }
        if (gameState.didJustEnterShuttleZone() && currentState == LEDState.TELEOP) {
            triggerBlip("SHUTTLE ZONE!", allianceColor);
        }

        // --- Pattern update ---
        // Per-LED patterns need full rate for smooth animation; simple patterns can throttle
        boolean needsFullRate = announcementActive || blipColor != null || gameDataLostActive
            || currentAction != ActionState.IDLE;
        ledUpdateCounter++;
        if (needsFullRate || ledUpdateCounter >= LED_UPDATE_DIVISOR) {
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
