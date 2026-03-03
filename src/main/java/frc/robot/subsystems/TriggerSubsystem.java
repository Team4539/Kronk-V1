package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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

    // Constructor
    
    /**
     * Creates and configures the trigger subsystem.
     * Uses Motion Magic Velocity for smooth, profiled speed control.
     */
    public TriggerSubsystem() {
        motor = new TalonFX(Constants.Trigger.MOTOR_ID);
        dutyControl = new DutyCycleOut(0);
        motionMagicVelocity = new MotionMagicVelocityVoltage(0).withSlot(0);
        
        configureMotor();
        
        // Initialize to stopped - only runs when trigger is pulled
        stop();
    }
    
    /**
     * Configures the trigger motor with brake mode and Motion Magic Velocity gains.
     */
    private void configureMotor() {
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
        targetRPS = Constants.Trigger.IDLE_SPEED_RPM / 60.0;
        isShooting = false;
    }
    
    /**
     * Set trigger to shooting mode (full speed to feed balls into shooter).
     */
    public void setShoot() {
        targetRPS = Constants.Trigger.SHOOT_SPEED_RPM / 60.0;
        isShooting = true;
    }
    
    /**
     * Stop the trigger completely.
     */
    public void stop() {
        targetRPS = 0.0;
        isShooting = false;
    }
    
    /**
     * Set trigger to a custom speed.
     * @param rpm Target RPM (negative = reverse)
     */
    public void setSpeed(double rpm) {
        targetRPS = rpm / 60.0;
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
     */
    @Override
    public void periodic() {
        // SAFETY: When robot is disabled, do NOT send any CAN frames.
        if (DriverStation.isDisabled()) {
            updateDashboard();
            return;
        }
        
        // Use Motion Magic Velocity for non-zero targets, duty cycle 0 for stop
        if (Math.abs(targetRPS) > 0.1) {
            motor.setControl(motionMagicVelocity.withVelocity(targetRPS));
        } else {
            motor.setControl(dutyControl.withOutput(0));
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
        
        // Detailed diagnostics every 10 cycles (200ms)
        if (telemetryCounter >= 10) {
            telemetryCounter = 0;
            DashboardHelper.putNumber(Category.DEBUG, "Trigger/TargetRPM", targetRPS * 60.0);
            DashboardHelper.putNumber(Category.DEBUG, "Trigger/ActualRPM", getCurrentSpeed());
        }
    }
}
