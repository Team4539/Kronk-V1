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
 * Rich, polished patterns that are easy to distinguish even through smoke/panels.
 *
 * PHYSICAL LAYOUT (back strip only — belly strip disconnected):
 *   Indices 0-7:   Onboard LEDs
 *   Indices 8-45:  Back/top strip (38 LEDs)
 *
 * PATTERN SUMMARY (each is instantly recognizable):
 *   DISABLED/IDLE  = Flowing ember: smooth orange/blue gradient with gold accents & sparkle
 *   AUTO           = Energy surge: bouncing white-hot pulse with alliance color glow trail
 *   TELEOP ACTIVE  = Pulse wave — bouncing glow that speeds up and widens with urgency
 *   TELEOP WAITING = Other alliance color pulse wave, blends to ours in last 5s
 *   ENDGAME        = Pulse wave starting at 50% urgency, ramps to frantic solid
 *   BROWNOUT       = Dim amber flicker (safety — highest priority)
 *   E-STOP         = MAXIMUM ALERT: triple red scanner + 12Hz strobe + flash-bangs + chaos
 *   GAME DATA      = "WE GO FIRST!" edge-racing build-up or confident wipe transition
 *   FIRING         = Fast white strobe (15 Hz)
 *   READY TO SHOOT = Solid bright GREEN ("PULL THE TRIGGER!")
 *   SPOOLING       = Orange breathing pulse ("warming up")
 *   INTAKING       = Solid yellow-green
 *   FORCE SHOOT    = Purple breathing (force shoot override active)
 *   DEFENSIVE      = Pulsing amber/gold (brake mode active)
 *   MATCH END      = Fireworks show: bursts → rainbow wave → gold sparkle rain
 *
 * PULSE WAVE: A bright glow bounces back and forth along the strip.
 * As urgency increases (0→1), the wave gets wider, faster, and the base
 * brightness rises until the strip is nearly solid. A second wave appears
 * at high urgency. At max urgency, rapid flashing pulses overlay everything.
 * Much more readable than the old comet chase — you immediately feel
 * "calm" vs "hurry up" from across the field.
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

    // =========================================================================
    // ENUMS
    // =========================================================================

    /**
     * LED states for robot operation phases.
     * TELEOP handles endgame internally via GamePhase — no separate ENDGAME state needed.
     */
    public enum LEDState {
        BOOT_WARMUP,    // Team colors aurora while CANdle warms up
        DISABLED,       // Flowing ember (pits / pre-match)
        AUTO,           // Energy surge pulse wave
        TELEOP,         // Pulse wave — speed/width/brightness vary by shift/endgame
        ENDGAME,        // (Unused — endgame handled inside TELEOP via GamePhase.END_GAME)
        MATCH_END,      // Fireworks show celebration
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
        NO_AUTO,     // No auto selected — angry red/orange strobe
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
        factor = Math.max(0.0, factor);
        // Gamma correction (gamma 1.8) for perceptually linear brightness fading.
        double gFactor = factor <= 1.0 ? Math.pow(factor, 1.8) : factor;
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
        double brightness;
        if (phase < 0.25) {
            double t = phase / 0.25;
            brightness = t * t * (3 - 2 * t);
        } else if (phase < 0.75) {
            double t = (phase - 0.25) / 0.5;
            brightness = 1.0 - t * t * (3 - 2 * t);
        } else {
            brightness = 0.0;
        }
        brightness = 0.12 + 0.88 * brightness;
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
    // PATTERN: AUTO — Energy Surge
    // =========================================================================

    /**
     * AUTO pattern: A bright energy pulse bounces back and forth along the strip.
     * White-hot leading edge with alliance-color glow trail. Looks like the robot
     * is actively running its autonomous program — purposeful and energetic.
     * Dim alliance-color wave ripples in the background for depth.
     */
    private void setAutoSurgePattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;

        // Bounce cycle: 1.2s per full sweep
        double cycleTime = 1.2;
        double phase = (t % cycleTime) / cycleTime;
        double wavePos = phase < 0.5 ? (phase * 2.0) : (1.0 - (phase - 0.5) * 2.0);
        double headPos = wavePos * (stripCount - 1);

        int glowWidth = 8;

        clearBuffer();

        // Onboard: pulse in sync with wave — brightest when wave is at center
        double onboardPulse = 0.3 + 0.7 * (1.0 - Math.abs(wavePos - 0.5) * 2.0);
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, scaleColor(allianceColor, onboardPulse));
        }

        // Strip: sweeping pulse with white-hot leading edge and alliance glow trail
        for (int i = 0; i < stripCount; i++) {
            double dist = Math.abs(i - headPos);

            if (dist < 1.5) {
                // White-hot head core
                double core = 1.0 - dist / 1.5;
                setLED(stripStart + i, lerpColor(allianceColor, Constants.LEDs.WHITE, core));
            } else if (dist < glowWidth) {
                // Alliance color glow fading from head
                double falloff = 1.0 - ((dist - 1.5) / (glowWidth - 1.5));
                double brightness = falloff * falloff * falloff;
                // Inner glow tinted slightly toward white
                int[] color;
                if (dist < 3.5) {
                    color = lerpColor(allianceColor, Constants.LEDs.WARM_WHITE, 0.3 * (1.0 - (dist - 1.5) / 2.0));
                } else {
                    color = allianceColor;
                }
                setLED(stripStart + i, scaleColor(color, brightness));
            } else {
                // Background: subtle rolling wave in dim alliance color
                double bgWave = 0.04 + 0.03 * Math.sin(i * 0.5 + t * 2.0);
                setLED(stripStart + i, scaleColor(allianceColor, bgWave));
            }
        }

        pushBuffer();
    }

    // =========================================================================
    // PATTERN: IDLE / DISABLED — Flowing Ember
    // =========================================================================

    /**
     * Smooth flowing gradient of team orange and blue with gold accents.
     * Multiple sine harmonics create rich, organic color movement.
     * Random sparkle overlay adds life without being chaotic.
     * Gentle global breathing keeps it alive.
     */
    private void setIdleEmberPattern() {
        double t = Timer.getFPGATimestamp();
        int[] orange = Constants.LEDs.TEAM_SAFETY_ORANGE;
        int[] blue = Constants.LEDs.TEAM_BLUE;
        int[] gold = Constants.LEDs.TEAM_GOLD;
        int onboard = Constants.LEDs.ONBOARD_LED_COUNT;
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;

        // Smooth scroll speed
        double scrollSpeed = 3.0;
        double scrollPos = t * scrollSpeed;

        // Global breathing overlay
        double breath = 0.55 + 0.45 * ((Math.sin(t * 0.7) + 1.0) / 2.0);

        clearBuffer();

        // Onboard: gentle phase-shifted orange/blue crossfade
        for (int i = 0; i < onboard && i < ledCount; i++) {
            double onboardPhase = Math.sin(t * 1.2 + i * 0.8) * 0.5 + 0.5;
            int[] c = lerpColor(orange, blue, onboardPhase);
            setLED(i, scaleColor(c, breath));
        }

        // Strip: smooth flowing gradient with multiple harmonics
        for (int i = 0; i < stripCount; i++) {
            // Primary flow: smooth sine blend between orange and blue
            double colorPhase = Math.sin((i + scrollPos) * 0.18) * 0.5 + 0.5;
            // Secondary harmonic moving the other direction for richness
            double colorPhase2 = Math.sin((i - scrollPos * 0.7) * 0.25 + 1.5) * 0.5 + 0.5;
            double blend = (colorPhase + colorPhase2) / 2.0;

            int[] baseColor = lerpColor(orange, blue, blend);

            // Gold accent: third harmonic that occasionally pushes toward gold
            double goldAmount = Math.max(0, Math.sin((i + t * 2.0) * 0.4) - 0.7) / 0.3;
            if (goldAmount > 0) {
                baseColor = lerpColor(baseColor, gold, goldAmount * 0.4);
            }

            // Per-LED shimmer for organic texture
            double shimmer = 0.8 + 0.2 * noise1D(i * 0.6 + t * 1.5);

            // Random sparkle: occasional bright warm-white pixels
            double sparkle = noise1D(i * 3.7 + t * 8.0);
            if (sparkle > 0.92) {
                double sparkleIntensity = (sparkle - 0.92) / 0.08;
                baseColor = lerpColor(baseColor, Constants.LEDs.WARM_WHITE, sparkleIntensity * 0.6);
            }

            setLED(stripStart + i, scaleColor(baseColor, breath * shimmer));
        }

        pushBuffer();
    }

    // =========================================================================
    // PATTERN: VICTORY — Fireworks Show
    // =========================================================================

    /**
     * Three-phase cycling celebration that looks genuinely impressive:
     *   Phase A: Firework bursts — expanding rings from random positions
     *   Phase B: Rainbow wave — smooth flowing spectrum
     *   Phase C: Gold sparkle rain over team color base
     * Cycles every 3 seconds for the full celebration duration.
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

        // Cycle through 3 phases
        double phaseDuration = 3.0;
        int phaseNum = ((int)(elapsed / phaseDuration)) % 3;

        clearBuffer();

        // Onboard: rapid team color cycle with white bursts
        int onboardCycle = ((int)(t * 10)) % 4;
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
            // === PHASE A: FIREWORK BURSTS ===
            // Multiple expanding ring bursts from pseudo-random positions
            for (int i = 0; i < stripCount; i++) {
                double brightness = 0.0;
                int[] burstColor = gold;

                // 4 simultaneous burst sources with staggered timing
                for (int b = 0; b < 4; b++) {
                    double burstCycle = t * 1.5 + b * 0.7;
                    double burstTime = burstCycle % 1.0;
                    double burstSeed = Math.floor(burstCycle) * 5.0 + b * 17.3;
                    double burstCenter = noise1D(burstSeed) * stripCount;
                    double burstRadius = burstTime * (stripCount * 0.4);
                    double burstFade = 1.0 - burstTime;

                    double distFromBurst = Math.abs(i - burstCenter);
                    if (distFromBurst < burstRadius && distFromBurst > burstRadius - 4) {
                        double ringBright = burstFade * (1.0 - (burstRadius - distFromBurst) / 4.0);
                        if (ringBright > brightness) {
                            brightness = ringBright;
                            switch (b % 3) {
                                case 0: burstColor = gold; break;
                                case 1: burstColor = white; break;
                                default: burstColor = (b % 2 == 0) ? orange : blue; break;
                            }
                        }
                    }
                }

                // Background sparkle
                double sparkle = noise1D(i * 2.1 + t * 12.0);
                if (sparkle > 0.85) {
                    double sparkBright = (sparkle - 0.85) / 0.15;
                    if (sparkBright > brightness) {
                        brightness = sparkBright;
                        burstColor = white;
                    }
                }

                brightness = Math.max(0.08, brightness);
                setLED(stripStart + i, scaleColor(burstColor, brightness));
            }
        } else if (phaseNum == 1) {
            // === PHASE B: RAINBOW WAVE ===
            // Smooth flowing spectrum — the crowd-pleaser
            for (int i = 0; i < stripCount; i++) {
                double hue = ((double) i / stripCount + t * 0.5) % 1.0;
                int[] rgb = hueToRGB(hue);
                double bright = 0.7 + 0.3 * Math.sin(i * 0.5 + t * 4.0);
                setLED(stripStart + i, scaleColor(rgb, bright));
            }
        } else {
            // === PHASE C: GOLD SPARKLE RAIN ===
            // Team color base with cascading bright gold/white sparkles
            for (int i = 0; i < stripCount; i++) {
                double colorBlend = Math.sin(i * 0.3 + t * 0.5) * 0.5 + 0.5;
                int[] baseColor = lerpColor(orange, blue, colorBlend);
                double baseBright = 0.25 + 0.15 * Math.sin(t * 1.5);

                // Three layers of sparkle rain at different speeds
                double rain1 = noise1D(i * 1.3 + t * 6.0);
                double rain2 = noise1D(i * 2.7 + t * 8.0 + 50);
                double rain3 = noise1D(i * 0.9 + t * 10.0 + 100);
                double maxRain = Math.max(rain1, Math.max(rain2, rain3));

                if (maxRain > 0.82) {
                    double sparkBright = (maxRain - 0.82) / 0.18;
                    int[] sparkColor;
                    if (maxRain == rain1) sparkColor = gold;
                    else if (maxRain == rain2) sparkColor = white;
                    else sparkColor = Constants.LEDs.WARM_WHITE;
                    setLED(stripStart + i, lerpColor(scaleColor(baseColor, baseBright), sparkColor, sparkBright));
                } else {
                    setLED(stripStart + i, scaleColor(baseColor, baseBright));
                }
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
     * The most excessive pattern possible. Triple red scanner sweep, 12Hz strobe
     * overlay, random white "flash-bang" bursts, aggressive pulsing background,
     * and 16Hz alternating chaos on onboard LEDs.
     * This should be visible and alarming from ACROSS THE VENUE.
     */
    private void setEStopPattern() {
        double t = Timer.getFPGATimestamp();
        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        int[] red = Constants.LEDs.ESTOP_COLOR;
        int[] white = Constants.LEDs.WHITE;
        int[] hotCore = Constants.LEDs.ESTOP_CORE;

        // 12Hz strobe overlay (faster than before)
        boolean strobeOn = ((int)(t * 12.0) % 2) == 0;

        // Random flash-bang: ~3 times per second, entire strip goes white for 1 frame
        boolean flashBang = noise1D(t * 47.0) > 0.88;

        // Aggressive background pulse (never fully off)
        double bgPulse = 0.12 + 0.30 * ((Math.sin(t * 6.0) + 1) / 2);
        int[] dimRed = scaleColor(red, bgPulse);

        // THREE scanner heads bouncing at different speeds for chaos
        double scanPos1 = triangleWave(t * 2.5);
        double scanPos2 = triangleWave(t * 2.5 + 0.33);
        double scanPos3 = triangleWave(t * 3.7 + 0.15); // Different speed = unpredictable
        double head1 = scanPos1 * (stripCount - 1);
        double head2 = scanPos2 * (stripCount - 1);
        double head3 = scanPos3 * (stripCount - 1);
        int scanWidth = Math.max(5, stripCount / 5);

        clearBuffer();

        // Onboard LEDs: 16Hz alternating red/white chaos, each LED independent
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            boolean thisOn = ((int)(t * 16.0 + i * 3.7) % 2) == 0;
            int[] c = thisOn ? red : (flashBang ? white : hotCore);
            ledBuffer[i][0] = (int)(c[0] * masterBrightness);
            ledBuffer[i][1] = (int)(c[1] * masterBrightness);
            ledBuffer[i][2] = (int)(c[2] * masterBrightness);
        }

        if (flashBang) {
            // FLASH-BANG: entire strip white
            for (int i = 0; i < stripCount; i++) {
                setLED(stripStart + i, white);
            }
        } else {
            // Triple scanner with glow + strobe overlay
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

    /** Triangle wave: 0→1→0 over one period. Used for scanner bounce. */
    private double triangleWave(double t) {
        double phase = t % 1.0;
        if (phase < 0) phase += 1.0;
        return phase < 0.5 ? (phase * 2.0) : (1.0 - (phase - 0.5) * 2.0);
    }

    // =========================================================================
    // PATTERN: GAME DATA ANNOUNCEMENT — "WE KNOW WHO GOES FIRST!"
    // =========================================================================

    /**
     * Dramatic announcement when game data arrives.
     *
     * WE GO FIRST:
     *   Phase 1 (0-1.0s): Edge race — LEDs light up from both ends racing to center
     *   Phase 2 (1.0-2.0s): Full explosion — rapid strobe alliance/white with shifting blocks
     *   Phase 3 (2.0-3.0s): Triumphant solid — bright alliance color with slow power pulse
     *
     * THEY GO FIRST:
     *   Phase 1 (0-0.8s): Their color sweeps across once (single wave wipe)
     *   Phase 2 (0.8-1.8s): Our color wipes over theirs from center outward
     *   Phase 3 (1.8-3.0s): Confident our-color breathing
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
            // === WE GO FIRST — DRAMATIC BUILD-UP + EXPLOSION ===

            if (progress < 0.33) {
                // PHASE 1: EDGE RACE — LEDs light up from both ends toward center
                double raceProgress = progress / 0.33; // 0→1
                int litCount = (int)(raceProgress * (stripCount / 2 + 1));

                for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
                    double flash = 0.3 + 0.7 * raceProgress;
                    setLED(i, scaleColor(primaryColor, flash));
                }

                for (int i = 0; i < stripCount; i++) {
                    int distFromLeft = i;
                    int distFromRight = stripCount - 1 - i;
                    int minEdgeDist = Math.min(distFromLeft, distFromRight);

                    if (minEdgeDist < litCount) {
                        // Lit: alliance color, brighter toward the racing edge
                        double edgeDist = litCount - minEdgeDist;
                        double brightness;
                        if (edgeDist <= 2) {
                            // Leading edge: white-hot
                            brightness = 1.0;
                            int[] c = lerpColor(primaryColor, Constants.LEDs.WHITE, 0.7);
                            setLED(stripStart + i, scaleColor(c, brightness));
                        } else {
                            brightness = 0.5 + 0.3 * Math.sin(t * 8.0 + i);
                            setLED(stripStart + i, scaleColor(primaryColor, brightness));
                        }
                    } else {
                        // Not yet reached
                        setLED(stripStart + i, scaleColor(primaryColor, 0.03));
                    }
                }
            } else if (progress < 0.67) {
                // PHASE 2: FULL EXPLOSION — rapid strobe with racing color blocks
                double strobeHz = 12.0;
                boolean showPrimary = ((int)(t * strobeHz * 2) % 2) == 0;

                for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
                    int[] c = ((i % 2 == 0) == showPrimary) ? primaryColor : secondaryColor;
                    setLED(i, c);
                }

                int blockSize = 3;
                int shift = (int)(t * 30) % (blockSize * 2);
                for (int i = 0; i < stripCount; i++) {
                    boolean isPrimary = ((i + shift) / blockSize) % 2 == 0;
                    setLED(stripStart + i, isPrimary ? primaryColor : secondaryColor);
                }
            } else {
                // PHASE 3: TRIUMPHANT SOLID — bright alliance color with slow power pulse
                double brightness = 0.8 + 0.2 * ((Math.sin(t * 3.0) + 1) / 2);
                int[] c = scaleColor(primaryColor, brightness);
                for (int i = 0; i < ledCount; i++) {
                    ledBuffer[i][0] = (int)(c[0] * masterBrightness);
                    ledBuffer[i][1] = (int)(c[1] * masterBrightness);
                    ledBuffer[i][2] = (int)(c[2] * masterBrightness);
                }
            }
        } else {
            // === THEY GO FIRST — ACKNOWLEDGE THEN PREPARE ===

            if (progress < 0.27) {
                // PHASE 1: Their color sweeps across (single directional wipe)
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
                // PHASE 2: Our color wipes over theirs from center outward
                double wipeProgress = (progress - 0.27) / 0.33;
                int center = stripCount / 2;
                int reach = (int)(wipeProgress * center);

                for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
                    double blend = wipeProgress;
                    setLED(i, lerpColor(primaryColor, secondaryColor, blend));
                }

                for (int i = 0; i < stripCount; i++) {
                    int distFromCenter = Math.abs(i - center);
                    if (distFromCenter <= reach) {
                        // Our color has arrived
                        if (distFromCenter >= reach - 2) {
                            setLED(stripStart + i, lerpColor(secondaryColor, Constants.LEDs.WHITE, 0.3));
                        } else {
                            setLED(stripStart + i, secondaryColor);
                        }
                    } else {
                        // Their color, fading
                        double fade = 0.7 * (1.0 - wipeProgress * 0.5);
                        setLED(stripStart + i, scaleColor(primaryColor, fade));
                    }
                }
            } else {
                // PHASE 3: Confident our-color breathing
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
     * A bright glow bounces back and forth along the strip. As urgency increases:
     *   - Wave gets WIDER (from 4 LEDs to 24+)
     *   - Wave moves FASTER (0.5 to 3 sweeps/sec)
     *   - Base brightness RISES (strip fills up toward solid)
     *   - At urgency > 0.6: a SECOND wave appears for extra energy
     *   - At urgency > 0.9: rapid pulse flash overlay
     *
     * Much more intuitive than the old comet chase — "slow gentle wave" immediately
     * reads as "chill", and "fast bright nearly-solid" reads as "HURRY UP".
     *
     * @param color    RGB color of the wave
     * @param urgency  0.0 = relaxed, 1.0 = maximum. Clamped to [0,1].
     */
    private void setPulseWave(int[] color, double urgency) {
        urgency = Math.max(0.0, Math.min(1.0, urgency));

        int stripStart = Constants.LEDs.STRIP_START;
        int stripCount = Constants.LEDs.STRIP_COUNT;
        double t = Timer.getFPGATimestamp();

        // Wave speed ramps with urgency
        double wavesPerSec = 0.5 + 2.5 * urgency;

        // Wave width: narrow at low urgency, wide at high
        double waveHalfWidth = 2.0 + 12.0 * urgency;

        // Base brightness: higher = more solid feel at high urgency
        double baseBright = 0.05 + 0.50 * urgency * urgency;

        // Wave peak brightness
        double peakBright = 0.6 + 0.4 * urgency;

        // Primary wave position (bouncing)
        double headLED = triangleWave(t * wavesPerSec) * (stripCount - 1);

        // Secondary wave appears at urgency > 0.6
        boolean dualWave = urgency > 0.6;
        double head2LED = 0;
        if (dualWave) {
            head2LED = triangleWave(t * wavesPerSec + 0.5) * (stripCount - 1);
        }

        // At urgency > 0.9, rapid pulse flash overlay
        double rapidPulse = 1.0;
        if (urgency > 0.9) {
            double pulseRate = 6.0 + 10.0 * ((urgency - 0.9) / 0.1);
            rapidPulse = 0.7 + 0.3 * ((Math.sin(t * pulseRate * 2 * Math.PI) + 1) / 2);
        }

        clearBuffer();

        // Onboard LEDs: average brightness between base and peak
        double onboardBright = (baseBright + peakBright) * 0.5 * rapidPulse;
        for (int i = 0; i < Constants.LEDs.ONBOARD_LED_COUNT && i < ledCount; i++) {
            setLED(i, scaleColor(color, onboardBright));
        }

        // Strip: bouncing wave(s) over a base glow
        for (int i = 0; i < stripCount; i++) {
            // Primary wave contribution
            double dist1 = Math.abs(i - headLED);
            double waveBright1 = Math.max(0, 1.0 - dist1 / waveHalfWidth);
            waveBright1 = waveBright1 * waveBright1; // Quadratic falloff

            // Secondary wave contribution
            double waveBright2 = 0;
            if (dualWave) {
                double dist2 = Math.abs(i - head2LED);
                waveBright2 = Math.max(0, 1.0 - dist2 / waveHalfWidth);
                waveBright2 = waveBright2 * waveBright2;
            }

            double totalWave = Math.min(1.0, waveBright1 + waveBright2);
            double brightness = baseBright + (peakBright - baseBright) * totalWave;
            brightness *= rapidPulse;

            // Tint wave peaks toward warm white for a "hot" center
            int[] c;
            if (totalWave > 0.7) {
                double whiteTint = (totalWave - 0.7) / 0.3 * 0.25;
                c = lerpColor(color, Constants.LEDs.WARM_WHITE, whiteTint);
            } else {
                c = color;
            }

            setLED(stripStart + i, scaleColor(c, brightness));
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
     *  2.  E-Stop (FMS/DS level — maximum alert)
     *  3.  Game Data Announcement (who goes first — 3s one-shot)
     *  4.  Firing (active scoring feedback — white strobe)
     *  5.  Ready to Shoot (green = pull trigger)
     *  6.  Spooling (orange breathing = warming up)
     *  7.  Intaking (solid yellow-green)
     *  8.  Force Shoot (purple breathing)
     *  9.  Defensive Brake (pulsing amber/gold)
     *  10. Match phase patterns (boot, disabled, auto, teleop wave, endgame, victory)
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

        // === PRIORITY 2: REAL E-STOP (FMS/DS level) — MAXIMUM ALERT ===
        if (isEStopped) {
            currentPatternName = "E-STOP!!!";
            setEStopPattern();
            return;
        }

        // === PRIORITY 2.5: SHIFT/ENDGAME WARNING FLASH — brief override ===
        if (warningFlashColor != null) {
            double elapsed = Timer.getFPGATimestamp() - warningFlashStart;
            if (elapsed < WARNING_FLASH_DURATION) {
                currentPatternName = warningFlashName;
                setStrobe(warningFlashColor, 10.0);
                return;
            }
            warningFlashColor = null;
        }

        // === PRIORITY 3: GAME DATA ANNOUNCEMENT — robot knows who goes first! ===
        if (announcementActive) {
            double elapsed = Timer.getFPGATimestamp() - announcementStartTime;
            if (elapsed < ANNOUNCEMENT_DURATION) {
                currentPatternName = announcementWeGoFirst ? "WE GO FIRST!!!" : "They go first...";
                setGameDataAnnouncement(elapsed, announcementWeGoFirst);
                return;
            }
            announcementActive = false;
        }

        // === PRIORITY 4: FIRING — balls are actively being fed ===
        if (currentAction == ActionState.FIRING) {
            currentPatternName = "FIRING!";
            setStrobe(Constants.LEDs.WHITE, 15.0);
            return;
        }

        // === PRIORITY 5: READY TO SHOOT — vivid GREEN with subtle life pulse ===
        if (currentAction == ActionState.SHOOTING) {
            currentPatternName = "READY - SHOOT!";
            double pulse = 0.88 + 0.12 * Math.sin(Timer.getFPGATimestamp() * 6.0);
            setSolidColor(scaleColor(Constants.LEDs.SHOOTING_COLOR, pulse));
            return;
        }

        // === PRIORITY 6: SPOOLING — orange breathing ===
        if (currentAction == ActionState.SPOOLING) {
            currentPatternName = "Spooling Up";
            setBreathing(Constants.LEDs.SPOOLING_COLOR, 1.0);
            return;
        }

        // === PRIORITY 7: INTAKING — yellow-green with subtle life pulse ===
        if (currentAction == ActionState.INTAKING) {
            currentPatternName = "Intaking";
            double pulse = 0.93 + 0.07 * Math.sin(Timer.getFPGATimestamp() * 5.0);
            setSolidColor(scaleColor(Constants.LEDs.INTAKING_COLOR, pulse));
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

        // === PRIORITY 10: NO AUTO — angry red/orange strobe while disabled ===
        if (currentAction == ActionState.NO_AUTO) {
            currentPatternName = "NO AUTO!!!";
            setStrobe(Constants.LEDs.NO_AUTO_COLOR, 4.0);
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
                currentPatternName = "Disabled (Ember)";
                setIdleEmberPattern();
                break;

            case AUTO:
                currentPatternName = "Auto (Energy Surge)";
                setAutoSurgePattern();
                break;

            case TELEOP:
                // No FMS timing (practice mode) — gentle wave
                if (matchTime < 0) {
                    currentPatternName = "Teleop (Practice)";
                    setPulseWave(allianceColor, 0.15);
                    break;
                }

                // Endgame (last 30 seconds) — starts urgent, ends at max
                if (phase == GamePhase.END_GAME) {
                    double endgameUrgency = (matchTime > 0)
                        ? 0.5 + 0.5 * (1.0 - matchTime / 30.0)
                        : 1.0;
                    currentPatternName = String.format("ENDGAME (%.0fs, urg=%.2f)", matchTime, endgameUrgency);
                    setPulseWave(allianceColor, endgameUrgency);
                    break;
                }

                // --- Normal shift-based teleop ---
                if (gameState.isOurAllianceActive() && phase != GamePhase.TRANSITION) {
                    // OUR SHIFT — wave fills up as our time runs out
                    double timeLeft = gameState.getTimeRemainingActive();
                    double activeUrgency = 1.0 - Math.min(1.0, timeLeft / 25.0);

                    // Last shift before endgame: boost urgency
                    if (matchTime > 0 && matchTime <= 55.0) {
                        double endgameApproach = 1.0 - ((matchTime - 30.0) / 25.0);
                        activeUrgency = Math.max(activeUrgency, endgameApproach * 0.5);
                    }

                    currentPatternName = String.format("Active (%.0fs left, urg=%.2f)", timeLeft, activeUrgency);
                    setPulseWave(allianceColor, activeUrgency);
                } else {
                    // WAITING for our shift
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
                currentPatternName = "Victory Fireworks!";
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
