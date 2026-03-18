package frc.robot.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.util.DashboardHelper.Category;
import frc.robot.util.Elastic.NotificationLevel;

/**
 * Periodic CAN bus health monitor for all devices on the robot.
 *
 * Checks every ~1 second (50 cycles), publishes per-device booleans to the
 * dashboard, and sends Elastic notifications on state transitions only
 * (no spam). Grouped by subsystem so a full CAN bus failure doesn't
 * flood 13 individual notifications.
 */
public class CANHealthMonitor {

    // Check interval
    private int counter = 0;
    private static final int CHECK_INTERVAL_CYCLES = 50; // ~1 second at 50Hz

    // Subsystem references (null if disabled)
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final TriggerSubsystem trigger;
    private final IntakeSubsystem intake;
    private final LEDSubsystem leds;

    // --- Previous health state for transition detection ---
    private boolean lastAllHealthy = true;

    // Drivetrain devices
    private boolean lastDrivetrainHealthy = true;
    private final boolean[] lastModuleHealthy = {true, true, true, true};
    private boolean lastPigeonHealthy = true;

    // Mechanisms
    private boolean lastShooterHealthy = true;
    private boolean lastTriggerHealthy = true;
    private boolean lastIntakeHealthy = true;
    private boolean lastCandleHealthy = true;

    // Module names for logging
    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

    // --- Current health state (readable by dashboard/LEDs) ---
    private boolean allHealthy = true;

    public CANHealthMonitor(CommandSwerveDrivetrain drivetrain,
                            ShooterSubsystem shooter,
                            TriggerSubsystem trigger,
                            IntakeSubsystem intake,
                            LEDSubsystem leds) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.trigger = trigger;
        this.intake = intake;
        this.leds = leds;
    }

    /**
     * Call every robot cycle (~20ms). Actual checks run every CHECK_INTERVAL_CYCLES.
     */
    public void update() {
        counter++;
        if (counter < CHECK_INTERVAL_CYCLES) return;
        counter = 0;

        boolean healthy = true;

        // === DRIVETRAIN (4 modules × 3 devices + Pigeon2) ===
        if (drivetrain != null) {
            boolean dtHealthy = true;

            for (int i = 0; i < 4; i++) {
                var module = drivetrain.getModule(i);
                TalonFX drive = (TalonFX) module.getDriveMotor();
                TalonFX steer = (TalonFX) module.getSteerMotor();
                CANcoder encoder = (CANcoder) module.getEncoder();

                boolean modOk = drive.isAlive() && steer.isAlive() && encoder.isConnected();

                if (!modOk && lastModuleHealthy[i]) {
                    // Build detail string for notification
                    String detail = "";
                    if (!drive.isAlive()) detail += "Drive ";
                    if (!steer.isAlive()) detail += "Steer ";
                    if (!encoder.isConnected()) detail += "Encoder ";
                    notify(NotificationLevel.ERROR,
                            "CAN: " + MODULE_NAMES[i] + " Module OFFLINE",
                            MODULE_NAMES[i] + " offline: " + detail.trim());
                } else if (modOk && !lastModuleHealthy[i]) {
                    notify(NotificationLevel.INFO,
                            "CAN: " + MODULE_NAMES[i] + " Recovered", "");
                }
                lastModuleHealthy[i] = modOk;
                DashboardHelper.putBoolean(Category.DEBUG,
                        "CAN/Drive/" + MODULE_NAMES[i], modOk);
                if (!modOk) dtHealthy = false;
            }

            // Pigeon2 (ParentDevice — uses isConnected(), not isAlive())
            boolean pigeonOk = drivetrain.getPigeon2().isConnected();
            if (!pigeonOk && lastPigeonHealthy) {
                notify(NotificationLevel.ERROR,
                        "CAN: Pigeon2 OFFLINE", "Gyro not responding on CAN bus");
            } else if (pigeonOk && !lastPigeonHealthy) {
                notify(NotificationLevel.INFO, "CAN: Pigeon2 Recovered", "");
            }
            lastPigeonHealthy = pigeonOk;
            DashboardHelper.putBoolean(Category.DEBUG, "CAN/Drive/Pigeon", pigeonOk);
            if (!pigeonOk) dtHealthy = false;

            // Summarize drivetrain
            if (!dtHealthy && lastDrivetrainHealthy) {
                System.out.println("[CAN Health] *** DRIVETRAIN CAN FAULT ***");
            }
            lastDrivetrainHealthy = dtHealthy;
            DashboardHelper.putBoolean(Category.DEBUG, "CAN/Drivetrain", dtHealthy);
            if (!dtHealthy) healthy = false;
        }

        // === SHOOTER ===
        if (shooter != null) {
            boolean ok = shooter.checkHealth();
            transition("Shooter", ok, lastShooterHealthy, "Shooter motor");
            lastShooterHealthy = ok;
            DashboardHelper.putBoolean(Category.DEBUG, "CAN/Shooter", ok);
            if (!ok) healthy = false;
        }

        // === TRIGGER ===
        if (trigger != null) {
            boolean ok = trigger.checkHealth();
            transition("Trigger", ok, lastTriggerHealthy, "Trigger motor");
            lastTriggerHealthy = ok;
            DashboardHelper.putBoolean(Category.DEBUG, "CAN/Trigger", ok);
            if (!ok) healthy = false;
        }

        // === INTAKE ===
        if (intake != null) {
            boolean ok = intake.checkHealth();
            transition("Intake", ok, lastIntakeHealthy, "Intake motor(s) or CANcoder");
            lastIntakeHealthy = ok;
            DashboardHelper.putBoolean(Category.DEBUG, "CAN/Intake", ok);
            if (!ok) healthy = false;
        }

        // === CANDLE ===
        if (leds != null) {
            boolean ok = leds.checkHealth();
            transition("CANdle", ok, lastCandleHealthy, "CANdle LED controller");
            lastCandleHealthy = ok;
            DashboardHelper.putBoolean(Category.DEBUG, "CAN/CANdle", ok);
            if (!ok) healthy = false;
        }

        // === SUMMARY ===
        allHealthy = healthy;
        DashboardHelper.putBoolean(Category.MATCH, "CAN/AllHealthy", allHealthy);

        // One-shot notification when overall health changes
        if (!allHealthy && lastAllHealthy) {
            System.out.println("[CAN Health] *** ONE OR MORE CAN DEVICES OFFLINE ***");
        } else if (allHealthy && !lastAllHealthy) {
            notify(NotificationLevel.INFO, "CAN: All Devices Healthy", "All CAN devices recovered");
            System.out.println("[CAN Health] All devices recovered");
        }
        lastAllHealthy = allHealthy;
    }

    /** Handles transition notifications for simple subsystems. */
    private void transition(String name, boolean nowHealthy, boolean wasHealthy, String desc) {
        if (!nowHealthy && wasHealthy) {
            notify(NotificationLevel.ERROR,
                    "CAN: " + name + " OFFLINE", desc + " not responding on CAN bus");
            System.out.println("[CAN Health] *** " + name.toUpperCase() + " OFFLINE ***");
        } else if (nowHealthy && !wasHealthy) {
            notify(NotificationLevel.INFO,
                    "CAN: " + name + " Recovered", desc + " back online");
            System.out.println("[CAN Health] " + name + " recovered");
        }
    }

    private void notify(NotificationLevel level, String title, String desc) {
        Elastic.sendNotification(new Elastic.Notification()
                .withLevel(level)
                .withTitle(title)
                .withDescription(desc)
                .withDisplaySeconds(level == NotificationLevel.ERROR ? 5.0 : 3.0));
    }

    // --- Getters ---

    public boolean isAllHealthy() { return allHealthy; }
    public boolean isDrivetrainHealthy() { return lastDrivetrainHealthy; }
    public boolean isShooterHealthy() { return lastShooterHealthy; }
    public boolean isTriggerHealthy() { return lastTriggerHealthy; }
    public boolean isIntakeHealthy() { return lastIntakeHealthy; }
    public boolean isCandleHealthy() { return lastCandleHealthy; }
    public boolean isPigeonHealthy() { return lastPigeonHealthy; }
    public boolean isModuleHealthy(int module) { return lastModuleHealthy[module]; }
}
