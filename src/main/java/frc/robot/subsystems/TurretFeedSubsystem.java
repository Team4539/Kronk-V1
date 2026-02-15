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
 * Controls the turret feed motor for ball management.
 * Normally runs slowly reverse to build up balls.
 * Runs fast forward when shooting to dump balls into turret.
 */
public class TurretFeedSubsystem extends SubsystemBase {
    
    // Hardware
    
    /** Kraken motor that drives the turret feed */
    private final TalonFX motor;
    
    /** Duty cycle control request for the motor */
    private final DutyCycleOut control;
    

    // State
    
    /** Current target speed (duty cycle -1.0 to 1.0) */
    private double targetSpeed = Constants.TurretFeed.IDLE_SPEED;
    
    /** Whether the turret feed is in shooting mode */
    private boolean isShooting = false;
    
    /** Telemetry counter - only publish detailed data every N cycles */
    private int telemetryCounter = 0;

    // Constructor
    
    /**
     * Creates and configures the turret feed subsystem.
     * Sets up motor with brake mode for precise control.
     */
    public TurretFeedSubsystem() {
        motor = new TalonFX(Constants.TurretFeed.MOTOR_ID);
        control = new DutyCycleOut(0);
        
        configureMotor();
        
        // Initialize to stopped - only runs when trigger is pulled
        stop();
    }
    
    /**
     * Configures the turret feed motor with brake mode.
     */
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Brake mode for precise control
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Set motor direction
        config.MotorOutput.Inverted = Constants.TurretFeed.MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
            
        motor.getConfigurator().apply(config);
    }

    // public boolean CheckHealth(){
    //      if(motor.isAlive() && motor.getDeviceTemp().getValueAsDouble() !=0) {
    //         return spindexerHealthy = true;
    //     }
    //     else
    //         return spindexerHealthy = false;
        
    // }

    // CONTROL METHODS
    
    /**
     * Set turret feed to idle mode (slow reverse to build up balls).
     */
    public void setIdle() {
        targetSpeed = Constants.TurretFeed.IDLE_SPEED;
        isShooting = false;
    }
    
    /**
     * Set turret feed to shooting mode (fast forward to dump balls).
     */
    public void setShoot() {
        targetSpeed = Constants.TurretFeed.SHOOT_SPEED;
        isShooting = true;
    }
    
    /**
     * Stop the turret feed completely.
     */
    public void stop() {
        targetSpeed = Constants.TurretFeed.STOP_SPEED;
        isShooting = false;
    }
    
    /**
     * Set turret feed to a custom speed.
     * @param speed Duty cycle from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
    }

    // STATUS METHODS
    
    /**
     * Check if turret feed is in shooting mode.
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
     * Updates SmartDashboard with turret feed telemetry.
     * Throttles less critical data to reduce NetworkTables traffic.
     */
    private void updateDashboard() {
        telemetryCounter++;
        
        // Essential info every cycle
        DashboardHelper.putBoolean(Category.TURRET_FEED, "Is Shooting", isShooting);
        
        // Detailed diagnostics every 10 cycles (200ms)
        if (telemetryCounter >= 10) {
            telemetryCounter = 0;
            DashboardHelper.putNumber(Category.TURRET_FEED, "Target Speed", targetSpeed);
            DashboardHelper.putNumber(Category.TURRET_FEED, "Current Speed", getCurrentSpeed());
            DashboardHelper.putNumber(Category.TURRET_FEED, "Motor Current (A)", motor.getSupplyCurrent().getValueAsDouble());
            DashboardHelper.putNumber(Category.TURRET_FEED, "Motor Temp (C)", motor.getDeviceTemp().getValueAsDouble());
        }
    }
}
