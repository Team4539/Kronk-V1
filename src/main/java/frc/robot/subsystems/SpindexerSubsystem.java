package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controls the spindexer motor for ball staging.
 * Normally runs slowly forward to stage balls.
 * Runs fast forward when shooting to feed balls into turret.
 */
public class SpindexerSubsystem extends SubsystemBase {
    
    // Hardware
    
    /** Kraken motor that drives the spindexer */
    private final TalonFX motor;
    
    /** Duty cycle control request for the motor */
    private final DutyCycleOut control;
    private boolean spindexerHealthy;
       // State
    
    /** Current target speed (duty cycle -1.0 to 1.0) */
    private double targetSpeed = Constants.Spindexer.IDLE_SPEED;
    
    /** Whether the spindexer is in shooting mode */
    private boolean isShooting = false;

    // Constructor
    
    /**
     * Creates and configures the spindexer subsystem.
     * Sets up motor with brake mode for precise control.
     */
    public SpindexerSubsystem() {
        motor = new TalonFX(Constants.Spindexer.MOTOR_ID);
        control = new DutyCycleOut(0);
        
        configureMotor();
        
        // Initialize to idle speed (slow forward)
        setIdle();
    }
    
    /**
     * Configures the spindexer motor with brake mode.
     */
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Brake mode for precise control
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Set motor direction
        config.MotorOutput.Inverted = Constants.Spindexer.MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
            
        motor.getConfigurator().apply(config);
    }
    public boolean checkHealth() {
        if(motor.isAlive() && motor.getDeviceTemp().getValueAsDouble() !=0) {
            return spindexerHealthy = true;
        }
        else
            return spindexerHealthy = false;
    }

    public boolean isHealthy(){
        return spindexerHealthy;
    }
    // CONTROL METHODS
    
    /**
     * Set spindexer to idle mode (slow forward to stage balls).
     */
    public void setIdle() {
        targetSpeed = Constants.Spindexer.IDLE_SPEED;
        isShooting = false;
    }
    
    /**
     * Set spindexer to shooting mode (fast forward to feed balls).
     */
    public void setShoot() {
        targetSpeed = Constants.Spindexer.SHOOT_SPEED;
        isShooting = true;
    }
    
    /**
     * Stop the spindexer completely.
     */
    public void stop() {
        targetSpeed = Constants.Spindexer.STOP_SPEED;
        isShooting = false;
    }
    
    /**
     * Set spindexer to a custom speed.
     * @param speed Duty cycle from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
    }

    // STATUS METHODS
    
    /**
     * Check if spindexer is in shooting mode.
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
        // Apply target speed to motor
        motor.setControl(control.withOutput(targetSpeed));
        
        // Update dashboard telemetry
        updateDashboard();
    }
    
    /**
     * Updates SmartDashboard with spindexer telemetry.
     */
    private void updateDashboard() {
        DashboardHelper.putNumber(Category.SPINDEXER, "Target Speed", targetSpeed);
        DashboardHelper.putNumber(Category.SPINDEXER, "Current Speed", getCurrentSpeed());
        DashboardHelper.putBoolean(Category.SPINDEXER, "Is Shooting", isShooting);
        DashboardHelper.putNumber(Category.SPINDEXER, "Motor Current (A)", motor.getSupplyCurrent().getValueAsDouble());
        DashboardHelper.putNumber(Category.SPINDEXER, "Motor Temp (C)", motor.getDeviceTemp().getValueAsDouble());
    }
}
