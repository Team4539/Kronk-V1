package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
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
 * Adapted for Phoenix 6 API which does not support individual LED addressing via setLEDs().
 * Uses built-in animations instead of custom pixel-by-pixel rendering.
 */
public class LEDSubsystem extends SubsystemBase {
    
    // Hardware
    private final CANdle candle;
    private final CANdleConfiguration candleConfig;
    
    // Master brightness (applied to colors before creating animations)
    private double masterBrightness = 1.0;
    
    // CANdle boot readiness
    private boolean candleReady = false;
    private double bootStartTime = 0;
    private static final double CANDLE_MIN_BOOT_SECONDS = 1.5;
    
    // State tracking
    private LEDState currentState = LEDState.BOOT_WARMUP;
    private ActionState currentAction = ActionState.IDLE;
    private String currentPatternName = "Initializing...";
    
    private int[] allianceColor = Constants.LEDs.BLUE_ALLIANCE;
    private boolean isEStopped = false;
    
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
    
    // Deduplication to prevent spamming CAN bus with same control request
    private String lastAppliedAnimation = "";

    public enum LEDState {
        BOOT_WARMUP,
        DISABLED,
        AUTO,
        TELEOP,
        ENDGAME,
        MATCH_END,
        BROWNOUT
    }
    
    public enum ActionState {
        IDLE,
        FIRING,
        SHOOTING,
        SPOOLING,
        INTAKING,
        DEFENSIVE,
        FORCE_SHOOT,
        BROWNOUT
    }
    
    public LEDSubsystem() {
        candle = new CANdle(Constants.CANIds.CANDLE);
        gameState = GameStateManager.getInstance();
        
        candleConfig = new CANdleConfiguration();
        candleConfig.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
        candleConfig.LED.StripType = StripTypeValue.GRB;
        
        bootStartTime = Timer.getFPGATimestamp();
        
        DashboardHelper.putNumber(Category.DEBUG, "LED/Master_Brightness", masterBrightness);
    }
    
    public void setState(LEDState newState) {
        if (currentState != newState) {
            currentState = newState;
        }
    }
    
    public LEDState getState() { return currentState; }
    public void setAction(ActionState action) { currentAction = action; }
    public ActionState getAction() { return currentAction; }
    public void clearAction() { currentAction = ActionState.IDLE; }
    
    public void triggerMatchEndCelebration() { setState(LEDState.MATCH_END); }

    // =========================================================================
    // HELPER METHODS
    // =========================================================================

    private int r(int[] color) { return (int)(color[0] * masterBrightness); }
    private int g(int[] color) { return (int)(color[1] * masterBrightness); }
    private int b(int[] color) { return (int)(color[2] * masterBrightness); }
    
    private static final int LED_START = 0;
    private static final int LED_END = Constants.LEDs.LED_COUNT - 1;

    private RGBWColor toColor(int[] color) {
        return new RGBWColor(r(color), g(color), b(color));
    }

    private void applySolid(int[] color) {
        String key = "Solid_" + color[0] + "_" + color[1] + "_" + color[2];
        if (checkDedup(key)) return;
        candle.setControl(new SolidColor(LED_START, LED_END).withColor(toColor(color)));
    }

    private void applyStrobe(int[] color, double speed) {
        String key = "Strobe_" + color[0] + "_" + color[1] + "_" + color[2] + "_" + speed;
        if (checkDedup(key)) return;
        candle.setControl(new StrobeAnimation(LED_START, LED_END).withColor(toColor(color)).withFrameRate(speed));
    }

    private void applyBreathing(int[] color, double speed) {
        String key = "Fade_" + color[0] + "_" + color[1] + "_" + color[2] + "_" + speed;
        if (checkDedup(key)) return;
        candle.setControl(new SingleFadeAnimation(LED_START, LED_END).withColor(toColor(color)).withFrameRate(speed));
    }

    // Fallback for flow/larson
    private void applyFlow(int[] color, double speed) { applyStrobe(color, speed * 2.0); }
    private void applyLarson(int[] color, double speed) { applyBreathing(color, speed); }

    private void applyRainbow(double speed) {
        String key = "Rainbow_" + speed;
        if (checkDedup(key)) return;
        candle.setControl(new RainbowAnimation(LED_START, LED_END).withBrightness(masterBrightness).withFrameRate(speed));
    }

    private void applyFire() {
        String key = "Fire";
        if (checkDedup(key)) return;
        candle.setControl(new FireAnimation(LED_START, LED_END).withBrightness(masterBrightness).withSparking(0.5).withCooling(0.5).withFrameRate(0.7));
    }

    private boolean checkDedup(String key) {
        if (lastAppliedAnimation.equals(key)) return true;
        lastAppliedAnimation = key;
        return false;
    }
    
    // =========================================================================
    // DISPATCH LOGIC
    // =========================================================================

    private void updateLEDPattern() {
        double matchTime = DriverStation.getMatchTime();
        GamePhase phase = gameState.getGamePhase();
        
        // 1. Brownout
        if (currentAction == ActionState.BROWNOUT || currentState == LEDState.BROWNOUT) {
            currentPatternName = "Brownout";
            applyStrobe(Constants.LEDs.BROWNOUT_SPARK, 10.0);
            return;
        }
        
        // 2. E-Stop
        if (isEStopped) {
            currentPatternName = "E-STOP";
            applyStrobe(Constants.LEDs.ESTOP_COLOR, 5.0);
            return;
        }
        
        // 3. Game Data Announcement
        if (announcementActive) {
            double elapsed = Timer.getFPGATimestamp() - announcementStartTime;
            if (elapsed < ANNOUNCEMENT_DURATION) {
                currentPatternName = announcementWeGoFirst ? "WE GO FIRST!" : "They go first...";
                applyStrobe(announcementWeGoFirst ? allianceColor : Constants.LEDs.WHITE, 15.0);
                return;
            }
            announcementActive = false;
        }
        
        // 4. Firing
        if (currentAction == ActionState.FIRING) {
            currentPatternName = "FIRING!";
            applyStrobe(Constants.LEDs.WHITE, 20.0);
            return;
        }

        // 4.5. Head Back Warning
        if (!gameState.isOurAllianceActive() && gameState.isHeadBackWarning()) {
             currentPatternName = "HEAD BACK!";
             applyStrobe(allianceColor, 4.0);
             return;
        }
        
        // 5. Ready to Shoot
        if (currentAction == ActionState.SHOOTING) {
            currentPatternName = "SHOOT!";
            applySolid(Constants.LEDs.SHOOTING_COLOR);
            return;
        }

        // 5.6 Green Light Pre-Shift
        if (!gameState.isOurAllianceActive() && gameState.isGreenLightPreShift()) {
             currentPatternName = "PRE-SHIFT GREEN";
             applyStrobe(Constants.LEDs.SHOOTING_COLOR, 8.0);
             return;
        }

        // 6. Spooling
        if (currentAction == ActionState.SPOOLING) {
            currentPatternName = "Spooling";
            applyBreathing(Constants.LEDs.SPOOLING_COLOR, 0.8);
            return;
        }
        
        // 6.5 Endgame (Last 30s)
        if (phase == GamePhase.END_GAME && matchTime > 0) {
            if (matchTime <= 10.0) {
                 currentPatternName = "ENDGAME PANIC";
                 applyStrobe(allianceColor, 10.0);
            } else {
                 currentPatternName = "ENDGAME";
                 applyFlow(allianceColor, 3.0);
            }
            return;
        }

        // 7. Intaking
        if (currentAction == ActionState.INTAKING) {
            currentPatternName = "Intaking";
            applySolid(Constants.LEDs.INTAKING_COLOR);
            return;
        }
        
        // 8. Force Shoot
        if (currentAction == ActionState.FORCE_SHOOT) {
            currentPatternName = "Force Shoot";
            applyBreathing(Constants.LEDs.AIMING_COLOR, 1.0);
            return;
        }
        
        // 9. Defensive
        if (currentAction == ActionState.DEFENSIVE) {
            currentPatternName = "Defensive";
            applyBreathing(Constants.LEDs.TEAM_GOLD, 0.5);
            return;
        }
        
        // 10. States
        switch (currentState) {
            case BOOT_WARMUP:
                currentPatternName = "Boot";
                applyRainbow(0.5);
                double timeSinceBoot = Timer.getFPGATimestamp() - bootStartTime;
                if (timeSinceBoot > Constants.LEDs.BOOT_ANIMATION_DURATION) {
                    setState(LEDState.DISABLED);
                }
                break;
            case DISABLED:
                currentPatternName = "Disabled";
                applyRainbow(0.2);
                break;
            case AUTO:
                currentPatternName = "Auto";
                // Flash alliance color
                applyStrobe(allianceColor, 2.0);
                break;
            case TELEOP:
                currentPatternName = "Teleop";
                if (matchTime < 0) {
                   // Practice - slow flow
                   applyFlow(allianceColor, 0.5);
                } else if (gameState.isOurAllianceActive() && phase != GamePhase.TRANSITION) {
                    // Our turn - fast flow to show urgency
                    applyFlow(allianceColor, 1.5);
                } else {
                    // Waiting - slow flow
                    applyFlow(allianceColor, 0.3);
                }
                break;
            case ENDGAME:
                currentPatternName = "Endgame";
                applyStrobe(allianceColor, 5.0);
                break;
            case MATCH_END:
                currentPatternName = "Victory";
                applyFire();
                break;
            default:
                applySolid(Constants.LEDs.OFF);
                break;
        }
    }

    @Override
    public void periodic() {
        if (!candleReady) {
            if (Timer.getFPGATimestamp() - bootStartTime < CANDLE_MIN_BOOT_SECONDS) return;
            candle.getConfigurator().apply(candleConfig);
            candleReady = true;
        }
        
        double newBrightness = DashboardHelper.getNumber(Category.DEBUG, "LED/Master_Brightness", masterBrightness);
        if (Math.abs(newBrightness - masterBrightness) > 0.01) {
            masterBrightness = Math.max(0.0, Math.min(1.0, newBrightness));
        }

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            allianceColor = (alliance.get() == Alliance.Red) ? Constants.LEDs.RED_ALLIANCE : Constants.LEDs.BLUE_ALLIANCE;
        }
        
        isEStopped = DriverStation.isEStopped();
        
        // Game Message Logic
        if (gameState.didJustReceiveGameMessage() && !announcementActive) {
            announcementActive = true;
            announcementStartTime = Timer.getFPGATimestamp();
            announcementWeGoFirst = gameState.doWeGoFirst();
            String who = announcementWeGoFirst ? "WE GO FIRST!" : "They go first.";
             Elastic.sendNotification(new Elastic.Notification()
                .withLevel(announcementWeGoFirst ? NotificationLevel.INFO : NotificationLevel.WARNING)
                .withTitle(who).withDisplaySeconds(5.0));
        }
        
        updateLEDPattern();
        
        DashboardHelper.putString(Category.DEBUG, "LED/Pattern", currentPatternName);
        DashboardHelper.putString(Category.DEBUG, "LED/State", currentState.name());
        
        // Notifications
        double matchTime = DriverStation.getMatchTime();
        if (isEStopped && !hasNotifiedEStop) {
            hasNotifiedEStop = true;
            Elastic.sendNotification(new Elastic.Notification().withLevel(NotificationLevel.ERROR).withTitle("E-STOP!").withDisplaySeconds(10.0));
        }
        if (matchTime > 0 && matchTime <= 30.0 && !hasNotifiedEndgame && matchTime > 28.0) {
            hasNotifiedEndgame = true;
             Elastic.sendNotification(new Elastic.Notification().withLevel(NotificationLevel.WARNING).withTitle("ENDGAME!").withDisplaySeconds(3.0));
        }
    }
}
