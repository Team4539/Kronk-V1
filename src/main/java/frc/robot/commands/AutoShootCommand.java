package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.GameStateManager.TargetMode;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.ActionState;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.ShootingCalculator;

/**
 * Fully automated shooting command for a fixed shooter.
 * Handles shooter spin-up and trigger feeding.
 * 
 * Since there is no turret, aiming must be done by rotating the robot
 * before or during this command. This command focuses on:
 *   1. Spinning up the shooter to the calculated RPM
 *   2. Feeding balls via the trigger when ready
 * 
 * Alliance-aware: only feeds balls when our scoring window is active
 * (or during green-light pre-shift / force-shoot override).
 */
public class AutoShootCommand extends Command {
    
    // SUBSYSTEMS & STATE MANAGER
    
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    @SuppressWarnings("unused")
    private final CommandSwerveDrivetrain drivetrain;
    private final GameStateManager gameState;
    private final LEDSubsystem leds;
    private final TriggerSubsystem trigger;
    private final ShootingCalculator shootingCalc;
    
    // STATE TRACKING
    
    private boolean shooterReady = false;
    private boolean linedUp = false;
    private boolean allianceActive = false;
    private double distanceToTarget = 0.0;
    private TargetMode currentTargetMode = TargetMode.DISABLED;
    
    private boolean hasNotifiedReady = false;
    private boolean wasAllianceActive = false;
    private boolean hasNotifiedHeadBack = false;
    private boolean hasNotifiedGreenLight = false;
    private boolean wasFiring = false;
    private int telemetryCounter = 0;
    
    // CONSTRUCTORS
    
    /**
     * Creates the auto shoot command (shooter only, no trigger motor).
     */
    public AutoShootCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        this(shooter, vision, null, null, null);
    }
    
    /**
     * Creates the full auto shoot command with all subsystems.
     */
    public AutoShootCommand(ShooterSubsystem shooter, VisionSubsystem vision,
                           LEDSubsystem leds, TriggerSubsystem trigger,
                           CommandSwerveDrivetrain drivetrain) {
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.leds = leds;
        this.trigger = trigger;
        this.gameState = GameStateManager.getInstance();
        this.shootingCalc = ShootingCalculator.getInstance();
        
        addRequirements(shooter, vision);
        if (trigger != null) {
            addRequirements(trigger);
        }
    }

    // COMMAND LIFECYCLE

    @Override
    public void initialize() {
        SmartDashboard.putString("AutoShoot/Status", "Initializing");
        shooterReady = false;
        linedUp = false;
        hasNotifiedReady = false;
        wasAllianceActive = false;
        hasNotifiedHeadBack = false;
        hasNotifiedGreenLight = false;
        wasFiring = false;
    }

    @Override
    public void execute() {
        // Always update state first so LEDs and notifications have current data
        currentTargetMode = gameState.getTargetMode();
        allianceActive = gameState.isOurAllianceActive();
        shooterReady = shooter.isReady();
        linedUp = Math.abs(shootingCalc.getAngleToTarget()) < Constants.Driver.AIM_TOLERANCE_DEG;
        distanceToTarget = shootingCalc.getDistance();

        if (!gameState.isAutoAimEnabled()) {
            SmartDashboard.putString("AutoShoot/Status", "AUTO-AIM DISABLED");
            shooter.stop();
            updateLEDState();
            publishTelemetry();
            return;
        }

        if (!vision.hasPoseEstimate()) {
            SmartDashboard.putString("AutoShoot/Status", "No Pose - Cannot Shoot");

            // Notify driver of vision loss
            if (telemetryCounter % 50 == 0) {
                Elastic.sendNotification(new Elastic.Notification()
                    .withLevel(NotificationLevel.ERROR)
                    .withTitle("AIM ERROR")
                    .withDescription("No Vision Target!")
                    .withDisplaySeconds(1.0));
            }

            telemetryCounter++;
            shooter.stop();
            stopTrigger();
            updateLEDState();
            publishTelemetry();
            return;
        }

        // Handle disabled mode (alliance inactive, no green light, no force shoot)
        if (currentTargetMode == TargetMode.DISABLED && !gameState.isGreenLightPreShift() && !gameState.isForceShootEnabled()) {
            if (gameState.isSpoolWarning() || gameState.isHeadBackWarning()) {
                // Pre-spool during the 1-10s before our shift
                SmartDashboard.putString("AutoShoot/Status",
                    String.format("SHIFT IN %.0fs - Spooling", gameState.getSecondsUntilOurNextShift()));
                setShooterRPM();
            } else {
                SmartDashboard.putString("AutoShoot/Status", "ALLIANCE INACTIVE - Waiting");
                shooter.stop();
                stopTrigger();
            }
            updateLEDState();
            publishTelemetry();
            return;
        }

        // Set shooter RPM from calculator
        setShooterRPM();

        // Control trigger motor - feed balls when shooter is spun up and alliance is active
        boolean firing = isReadyToFire();
        if (firing) {
            if (trigger != null) {
                trigger.setShoot();
            }
        } else {
            idleTrigger();
        }
        wasFiring = firing;

        updateLEDState();
        updateNotifications();

        String status = determineStatus();
        SmartDashboard.putString("AutoShoot/Status", status);
        publishTelemetry();
        telemetryCounter++;
    }
    
    private void updateNotifications() {
        if (isReadyToFire() && !hasNotifiedReady) {
            hasNotifiedReady = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle("READY TO FIRE!")
                .withDescription(String.format("Distance: %.1fm | %s", distanceToTarget, 
                    currentTargetMode == TargetMode.TRENCH ? "Trench" : "Hub"))
                .withDisplaySeconds(1.5));
        } else if (!isReadyToFire()) {
            hasNotifiedReady = false;
        }
        
        if (allianceActive && !wasAllianceActive) {
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle("Alliance Window OPEN")
                .withDescription("Scoring is now allowed!")
                .withDisplaySeconds(2.0));
            hasNotifiedHeadBack = false;
            hasNotifiedGreenLight = false;
        } else if (!allianceActive && wasAllianceActive && !gameState.isForceShootEnabled()) {
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle("Alliance Window CLOSED")
                .withDescription("Waiting for next window...")
                .withDisplaySeconds(2.0));
        }
        wasAllianceActive = allianceActive;
        
        if (gameState.isHeadBackWarning() && !hasNotifiedHeadBack) {
            hasNotifiedHeadBack = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle("<- HEAD BACK!")
                .withDescription(String.format("Our shift starts in %.0fs - get in position",
                    gameState.getSecondsUntilOurNextShift()))
                .withDisplaySeconds(2.0));
        } else if (!gameState.isHeadBackWarning()) {
            hasNotifiedHeadBack = false;
        }

        if (gameState.isGreenLightPreShift() && !hasNotifiedGreenLight) {
            hasNotifiedGreenLight = true;
            String desc = gameState.isShootNowWarning()
                    ? "FIRE! Shift starts in 1 second!"
                    : String.format("Spool up! Shift in %.0fs", gameState.getSecondsUntilOurNextShift());
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.INFO)
                .withTitle("GREEN LIGHT!")
                .withDescription(desc)
                .withDisplaySeconds(2.0));
        } else if (!gameState.isGreenLightPreShift()) {
            hasNotifiedGreenLight = false;
        }
    }
    
    private void updateLEDState() {
        if (leds == null) return;

        if (!vision.hasPoseEstimate()) {
            // Signal error when vision is lost while trying to shoot
            leds.setAction(ActionState.BROWNOUT);
        } else if (isReadyToFire()) {
            // Trigger is actively feeding balls
            leds.setAction(ActionState.FIRING);
        } else if (shooterReady && linedUp) {
            // Shooter at speed + robot aimed but can't fire yet (alliance inactive)
            // This is the "SHOOT NOW" green indicator the driver needs to see
            leds.setAction(ActionState.SHOOTING);
        } else {
            // Still spinning up or not aimed yet
            leds.setAction(ActionState.SPOOLING);
        }
    }
    
    // TRIGGER CONTROL
    
    private void idleTrigger() {
        if (trigger != null) {
            trigger.setIdle();
        }
    }
    
    private void stopTrigger() {
        if (trigger != null) {
            trigger.stop();
        }
    }
    
    // SHOOTER CONTROL
    
    private void setShooterRPM() {
        double rpm = shootingCalc.getTargetRPM();
        shooter.setTargetRPM(rpm);
    }
    
    // STATUS
    
    private String determineStatus() {
        if (gameState.isGreenLightPreShift()) {
            return String.format("GREEN LIGHT! Shift in %.1fs", gameState.getSecondsUntilOurNextShift());
        }
        if (gameState.isHeadBackWarning()) {
            return String.format("HEAD BACK - Shift in %.1fs", gameState.getSecondsUntilOurNextShift());
        }
        if (!allianceActive && !gameState.isForceShootEnabled()) {
            return String.format("WAITING - Active in %.1fs", gameState.getTimeUntilActive());
        }
        
        String targetStr = (currentTargetMode == TargetMode.TRENCH) ? "[TRENCH] " : "[HUB] ";
        if (!shooterReady) {
            return targetStr + "Spinning Up...";
        } else if (!linedUp) {
            return targetStr + String.format("AIMING (%.1f\u00b0 off)", shootingCalc.getAngleToTarget());
        } else {
            return targetStr + "READY!";
        }
    }
    
    private void publishTelemetry() {
        SmartDashboard.putBoolean("AutoShoot/ReadyToFire", isReadyToFire());
        SmartDashboard.putBoolean("AutoShoot/LinedUp", linedUp);
        SmartDashboard.putNumber("AutoShoot/AimError", shootingCalc.getAngleToTarget());
        SmartDashboard.putNumber("AutoShoot/Distance", distanceToTarget);
    }
    
    public boolean isReadyToFire() {
        return shooterReady &&
               (allianceActive || gameState.isForceShootEnabled() || gameState.isGreenLightPreShift());
    }
    
    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        stopTrigger();
        if (leds != null) {
            leds.clearAction();
        }
        SmartDashboard.putString("AutoShoot/Status", interrupted ? "Interrupted" : "Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
