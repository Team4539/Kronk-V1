package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controls the trigger motor that feeds balls from the intake
 * into the fixed shooter. Runs at idle speed normally, and at
 * full shoot speed to feed balls into the shooter when firing.
 */
public class TriggerSubsystem extends SubsystemBase {
    
    // Hardware
    
    /** Kraken motor that drives the trigger feed */
    private final TalonFX motor;
    
    /** Duty cycle control request for the motor */
    private final DutyCycleOut control;

    // State
    
    /** Current target speed (duty cycle -1.0 to 1.0) */
    private double targetSpeed = Constants.Trigger.IDLE_SPEED;
    
    /** Whether the trigger is in shooting mode */
    private boolean isShooting = false;
    
    /** Telemetry counter - only publish detailed data every N cycles */
    private int telemetryCounter = 0;

    // Constructor
    
    /**
     * Creates and configures the trigger subsystem.
     * Sets up motor with brake mode for precise control.
     */
    public TriggerSubsystem() {
        motor = new TalonFX(Constants.Trigger.MOTOR_ID);
        control = new DutyCycleOut(0);
        
        configureMotor();
        
        // Initialize to stopped - only runs when trigger is pulled
        stop();
    }
    
    /**
     * Configures the trigger motor with brake mode.
     */
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Brake mode for precise control
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Set motor direction
        config.MotorOutput.Inverted = Constants.Trigger.MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
            
        motor.getConfigurator().apply(config);
    }

    /** Check if the trigger motor is responding on the CAN bus. */
    public boolean checkHealth() {
        return motor.isAlive();
    }

    // CONTROL METHODS
    
    /**
     * Set trigger to idle mode (slow speed to stage balls near the shooter).
     */
    public void setIdle() {
        targetSpeed = Constants.Trigger.IDLE_SPEED;
        isShooting = false;
    }
    
    /**
     * Set trigger to shooting mode (full speed to feed balls into shooter).
     */
    public void setShoot() {
        targetSpeed = Constants.Trigger.SHOOT_SPEED;
        isShooting = true;
    }
    
    /**
     * Stop the trigger completely.
     */
    public void stop() {
        targetSpeed = Constants.Trigger.STOP_SPEED;
        isShooting = false;
    }
    
    /**
     * Set trigger to a custom speed.
     * @param speed Duty cycle from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
    }

    // STATUS METHODS
    
    /**
     * Check if trigger is in shooting mode.
     * @return True if shooting, false otherwise
     */
    public boolean isShooting() {
        return isShooting;
    }
    
    /**
     * Get the current target speed.
     * @return Target speed (duty cycle -1.0 to 1.0)
     */
    public double getTargetSpeed() {
        return targetSpeed;
    }
    
    /**
     * Get the current motor speed.
     * @return Actual motor speed (duty cycle -1.0 to 1.0)
     */
    public double getCurrentSpeed() {
        return motor.get();
    }

    // PERIODIC
    
    /**
     * Called every robot loop (20ms).
     * Updates motor output and telemetry.
     */
    @Override
    public void periodic() {
        // SAFETY: When robot is disabled, do NOT send any CAN frames.
        if (DriverStation.isDisabled()) {
            updateDashboard();
            return;
        }
        
        // Apply target speed to motor
        motor.setControl(control.withOutput(targetSpeed));
        
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
        
        // Detailed diagnostics every 10 cycles (200ms)
        if (telemetryCounter >= 10) {
            telemetryCounter = 0;
            DashboardHelper.putNumber(Category.DEBUG, "Trigger/TargetSpeed", targetSpeed);
            DashboardHelper.putNumber(Category.DEBUG, "Trigger/CurrentSpeed", getCurrentSpeed());
        }
    }
}
