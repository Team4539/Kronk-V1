
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controls the trigger motor that feeds balls from the intake
 * into the fixed shooter. Runs at idle speed normally, and at
 * full shoot speed to feed balls into the shooter when firing.
 * 
 * Includes stall detection: if the trigger jams, it will automatically
 * reverse at full speed to spit the ball back, then resume normal operation.
 */
public class TriggerSubsystem extends SubsystemBase {
    
    // Hardware
    
    /** Kraken motor that drives the trigger feed */
    private final TalonFX motor;
    
    /** Duty cycle control for stopped state */
    private final DutyCycleOut dutyControl;
    
    /** Motion Magic Velocity control for smooth speed profiling */
    private final MotionMagicVelocityVoltage motionMagicVelocity;

    // State
    
    /** Current target speed in RPS (rotations per second) */
    private double targetRPS = 0.0;
    
    /** Whether the trigger is in shooting mode */
    private boolean isShooting = false;
    
    /** Telemetry counter - only publish detailed data every N cycles */
    private int telemetryCounter = 0;
    
    /** Flag to prevent spam logging when robot is disabled */
    private boolean hasLoggedDisabled = false;
    
    // Stall detection state machine
    private enum StallState { NORMAL, REVERSING }
    private StallState stallState = StallState.NORMAL;
    /** Timestamp when the RPM first dropped to ~0 while commanded to spin */
    private double stallStartTime = 0.0;
    /** Timestamp when the reverse started */
    private double reverseStartTime = 0.0;

    // Constructor
    
    /**
     * Creates and configures the trigger subsystem.
     * Uses Motion Magic Velocity for smooth, profiled speed control.
     */
    public TriggerSubsystem() {
        System.out.println("[Trigger] Initializing TriggerSubsystem...");
        
        motor = new TalonFX(Constants.Trigger.MOTOR_ID);
        dutyControl = new DutyCycleOut(0);
        motionMagicVelocity = new MotionMagicVelocityVoltage(0).withSlot(0);
        
        configureMotor();
        
        // Initialize to stopped - only runs when trigger is pulled
        stop();
        
        System.out.println("[Trigger] TriggerSubsystem initialized successfully. Motor ID: " + Constants.Trigger.MOTOR_ID);
    }
    
    /**
     * Configures the trigger motor with brake mode and Motion Magic Velocity gains.
     */
    private void configureMotor() {
        System.out.println("[Trigger] Configuring motor settings...");
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Brake mode for precise control
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Set motor direction
        config.MotorOutput.Inverted = Constants.Trigger.MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        
        // Slot 0: velocity PID gains for Motion Magic Velocity
        config.Slot0.kV = 0.12;
        config.Slot0.kP = 0.15;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.05;
        
        // Motion Magic: acceleration & jerk limits
        config.MotionMagic.MotionMagicAcceleration = 200;  // rps/s
        config.MotionMagic.MotionMagicJerk = 2000;         // rps/s²
            
        motor.getConfigurator().apply(config);
        System.out.println("[Trigger] Motor configuration applied. PID: kV=0.12, kP=0.15, Accel=200rps/s");
    }

    /** Check if the trigger motor is responding on the CAN bus. */
    public boolean checkHealth() {
        boolean isAlive = motor.isAlive();
        if (!isAlive) {
            System.out.println("[Trigger] *** HEALTH CHECK FAILED *** Motor ID " + Constants.Trigger.MOTOR_ID + " not responding!");
        }
        return isAlive;
    }

    // CONTROL METHODS
    
    /**
     * Set trigger to idle mode (slow speed to stage balls near the shooter).
     */
    public void setIdle() {
        double newTargetRPS = Constants.Trigger.IDLE_SPEED_RPM / 60.0;
        if (Math.abs(newTargetRPS - targetRPS) > 0.1) {
            System.out.println("[Trigger] Setting IDLE mode: " + Constants.Trigger.IDLE_SPEED_RPM + " RPM");
        }
        targetRPS = newTargetRPS;
        isShooting = false;
    }
    
    /**
     * Set trigger to shooting mode (full speed to feed balls into shooter).
     */
    public void setShoot() {
        double newTargetRPS = Constants.Trigger.SHOOT_SPEED_RPM / 60.0;
        if (Math.abs(newTargetRPS - targetRPS) > 0.1) {
            System.out.println("[Trigger] Setting SHOOT mode: " + Constants.Trigger.SHOOT_SPEED_RPM + " RPM");
        }
        targetRPS = newTargetRPS;
        isShooting = true;
    }
    
    /**
     * Stop the trigger completely.
     */
    public void stop() {
        if (Math.abs(targetRPS) > 0.1 || stallState != StallState.NORMAL) {
            System.out.println("[Trigger] STOPPING - was at " + (targetRPS * 60.0) + " RPM, stallState: " + stallState);
        }
        targetRPS = 0.0;
        isShooting = false;
        stallState = StallState.NORMAL;
        stallStartTime = 0.0;
    }
    
    /**
     * Set trigger to a custom speed.
     * @param rpm Target RPM (negative = reverse)
     */
    public void setSpeed(double rpm) {
        double newTargetRPS = rpm / 60.0;
        if (Math.abs(newTargetRPS - targetRPS) > 0.1) {
            System.out.println("[Trigger] Setting custom speed: " + rpm + " RPM");
        }
        targetRPS = newTargetRPS;
    }

    // STATUS METHODS
    
    /**
     * Check if trigger is in shooting mode.
     * @return True if shooting, false otherwise
     */
    public boolean isShooting() {
        return isShooting;
    }

    public boolean isStalled() {
        return stallState == StallState.REVERSING;
    }

    /**
     * Get the current target speed in RPM.
     * @return Target speed in RPM
     */
    public double getTargetSpeed() {
        return targetRPS * 60.0;
    }
    
    /**
     * Get the current motor speed in RPM.
     * @return Actual motor speed in RPM
     */
    public double getCurrentSpeed() {
        return motor.getVelocity().getValueAsDouble() * 60.0;
    }

    // PERIODIC
    
    /**
     * Called every robot loop (20ms).
     * Updates motor output and telemetry.
     * Includes stall detection — if jammed, reverses to spit back.
     */
    @Override
    public void periodic() {
        // SAFETY: When robot is disabled, explicitly command zero output.
        // Just returning without a setControl() leaves the last command active.
        if (DriverStation.isDisabled()) {
            if (!hasLoggedDisabled && (Math.abs(targetRPS) > 0.1 || stallState != StallState.NORMAL)) {
                System.out.println("[Trigger] Robot disabled - stopping motor and resetting stall state");
                hasLoggedDisabled = true;
            }
            
            motor.setControl(dutyControl.withOutput(0));
            targetRPS = 0.0;
            isShooting = false;
            stallState = StallState.NORMAL;
            stallStartTime = 0.0;
            updateDashboard();
            return;
        } else if (hasLoggedDisabled) {
            // Robot just re-enabled
            System.out.println("[Trigger] Robot re-enabled - ready for operation");
            hasLoggedDisabled = false;
        }
        
        double now = Timer.getFPGATimestamp();
        double actualRPS = motor.getVelocity().getValueAsDouble();
        double actualRPM = actualRPS * 60.0;
        boolean isCommanded = Math.abs(targetRPS) > 0.1;
        
        // === STALL DETECTION STATE MACHINE ===
        switch (stallState) {
            case NORMAL:
                // Detect stall: motor is commanded to spin but actual RPM is near zero
                if (isCommanded && Math.abs(actualRPM) < Constants.Trigger.STALL_RPM_THRESHOLD) {
                    if (stallStartTime == 0.0) {
                        stallStartTime = now;  // Start the stall timer
                        System.out.println("[Trigger] STALL detection started - actualRPM: " + String.format("%.1f", actualRPM) + 
                                         ", targetRPM: " + String.format("%.1f", targetRPS * 60.0) + 
                                         ", threshold: " + Constants.Trigger.STALL_RPM_THRESHOLD);
                    } else if (now - stallStartTime >= Constants.Trigger.STALL_TIME_SECONDS) {
                        // Confirmed stall — switch to reversing (spit back)
                        stallState = StallState.REVERSING;
                        reverseStartTime = now;
                        stallStartTime = 0.0;
                        System.out.println("[Trigger] *** STALL CONFIRMED *** Switching to REVERSE mode at " + 
                                         Constants.Trigger.STALL_REVERSE_RPS + " RPS for " + 
                                         Constants.Trigger.STALL_REVERSE_TIME_SECONDS + "s");
                        Elastic.sendNotification(new Elastic.Notification()
                            .withLevel(NotificationLevel.WARNING)
                            .withTitle("Trigger JAM!")
                            .withDescription("Stall detected — spitting back")
                            .withDisplaySeconds(2.0));
                    } else {
                        // Still in stall detection window
                        double timeRemaining = Constants.Trigger.STALL_TIME_SECONDS - (now - stallStartTime);
                        if ((int)(timeRemaining * 10) % 5 == 0) {  // Print every 0.5s during stall detection
                            System.out.println("[Trigger] Stall detection: " + String.format("%.2f", timeRemaining) + "s remaining");
                        }
                    }
                } else {
                    if (stallStartTime != 0.0) {
                        System.out.println("[Trigger] Stall detection CLEARED - motor spinning normally at " + 
                                         String.format("%.1f", actualRPM) + " RPM");
                    }
                    stallStartTime = 0.0;  // Reset — motor is spinning fine
                }
                
                // Normal motor control
                if (isCommanded) {
                    motor.setControl(motionMagicVelocity.withVelocity(targetRPS));
                } else {
                    motor.setControl(dutyControl.withOutput(0));
                }
                break;
                
            case REVERSING:
                // Full speed reverse (spit back) to clear the jam
                motor.setControl(motionMagicVelocity.withVelocity(Constants.Trigger.STALL_REVERSE_RPS));
                
                double timeRemaining = Constants.Trigger.STALL_REVERSE_TIME_SECONDS - (now - reverseStartTime);
                if ((int)(timeRemaining * 10) % 10 == 0) {  // Print every 0.1s during reverse
                    System.out.println("[Trigger] REVERSING to clear jam: " + String.format("%.2f", timeRemaining) + "s remaining, " +
                                     "actualRPM: " + String.format("%.1f", actualRPM));
                }
                
                if (now - reverseStartTime >= Constants.Trigger.STALL_REVERSE_TIME_SECONDS) {
                    // Done reversing — go back to normal
                    System.out.println("[Trigger] REVERSE complete - returning to NORMAL mode, resuming target: " + 
                                     String.format("%.1f", targetRPS * 60.0) + " RPM");
                    stallState = StallState.NORMAL;
                    stallStartTime = 0.0;
                }
                break;
        }
        
        // Update dashboard telemetry
        updateDashboard();
    }
    
    /**
     * Updates SmartDashboard with trigger telemetry.
     * Throttles less critical data to reduce NetworkTables traffic.
     */
    private void updateDashboard() {
        telemetryCounter++;
        
        // Essential info every cycle
        DashboardHelper.putBoolean(Category.DEBUG, "Trigger/IsShooting", isShooting);
        DashboardHelper.putBoolean(Category.DEBUG, "Trigger/Stalled", stallState == StallState.REVERSING);
        DashboardHelper.putString(Category.DEBUG, "Trigger/State", stallState.toString());
        
        // Detailed diagnostics every 10 cycles (200ms)
        if (telemetryCounter >= 10) {
            telemetryCounter = 0;
            double actualRPM = getCurrentSpeed();
            DashboardHelper.putNumber(Category.DEBUG, "Trigger/TargetRPM", targetRPS * 60.0);
            DashboardHelper.putNumber(Category.DEBUG, "Trigger/ActualRPM", actualRPM);
            DashboardHelper.putNumber(Category.DEBUG, "Trigger/RPMError", Math.abs(actualRPM - (targetRPS * 60.0)));
            DashboardHelper.putNumber(Category.DEBUG, "Trigger/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
            
            // Log significant RPM errors (only when commanded to move)
            if (Math.abs(targetRPS) > 0.1) {
                double error = Math.abs(actualRPM - (targetRPS * 60.0));
                if (error > 100.0) {  // More than 100 RPM error
                    System.out.println("[Trigger] Large RPM error: target=" + String.format("%.1f", targetRPS * 60.0) + 
                                     " actual=" + String.format("%.1f", actualRPM) + 
                                     " error=" + String.format("%.1f", error) + 
                                     " current=" + String.format("%.1f", motor.getSupplyCurrent().getValueAsDouble()) + "A");
                }
            }
        }
    }
}

/*
 * LOGGING SUMMARY for TriggerSubsystem:
 * 
 * INITIALIZATION:
 * - "[Trigger] Initializing TriggerSubsystem..."
 * - "[Trigger] TriggerSubsystem initialized successfully. Motor ID: X"
 * - "[Trigger] Configuring motor settings..."
 * - "[Trigger] Motor configuration applied. PID: kV=0.12, kP=0.15, Accel=200rps/s"
 * 
 * COMMAND CHANGES:
 * - "[Trigger] Setting IDLE mode: X RPM"
 * - "[Trigger] Setting SHOOT mode: X RPM" 
 * - "[Trigger] Setting custom speed: X RPM"
 * - "[Trigger] STOPPING - was at X RPM, stallState: X"
 * 
 * ROBOT STATE:
 * - "[Trigger] Robot disabled - stopping motor and resetting stall state"
 * - "[Trigger] Robot re-enabled - ready for operation"
 * 
 * STALL DETECTION:
 * - "[Trigger] STALL detection started - actualRPM: X, targetRPM: X, threshold: X"
 * - "[Trigger] Stall detection: Xs remaining"
 * - "[Trigger] *** STALL CONFIRMED *** Switching to REVERSE mode at X RPS for Xs"
 * - "[Trigger] REVERSING to clear jam: Xs remaining, actualRPM: X"
 * - "[Trigger] REVERSE complete - returning to NORMAL mode, resuming target: X RPM"
 * - "[Trigger] Stall detection CLEARED - motor spinning normally at X RPM"
 * 
 * PERFORMANCE MONITORING:
 * - "[Trigger] Large RPM error: target=X actual=X error=X current=XA" (every 200ms if error >100 RPM)
 * 
 * HEALTH CHECKS:
 * - "[Trigger] *** HEALTH CHECK FAILED *** Motor ID X not responding!"
 */
