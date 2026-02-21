package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Dual flywheel shooter subsystem with two independently controlled motors.
 * 
 * Top and bottom motors spin at different RPMs to control shot trajectory:
 * - Differential spin creates backspin/topspin for arc control
 * - Higher RPM = more distance
 * 
 * Primary control is via closed-loop velocity (RPM) using TalonFX VelocityVoltage.
 * Manual duty-cycle control is available for calibration.
 * Distance-to-RPM interpolation is handled by ShootingCalculator.
 */
public class ShooterSubsystem extends SubsystemBase {
    
    // Hardware
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private final DutyCycleOut topControl;
    private final DutyCycleOut bottomControl;
    private final VelocityVoltage topVelocityControl;
    private final VelocityVoltage bottomVelocityControl;

    // Hardware Healthy
    public boolean isHealthy;
    
    // State
    private double targetTopPower = 0.0;
    private double targetBottomPower = 0.0;
    private double targetTopRPS = 0.0;    // Target velocity in rotations per second
    private double targetBottomRPS = 0.0; // Target velocity in rotations per second
    private boolean useVelocityControl = false; // Whether to use velocity PID vs duty cycle
    private double currentDistance = 0.0;
    private boolean isSpunUp = false;
    private long spinUpStartTime = 0;
    
    /** RPM tolerance for considering the shooter "at speed" (±RPM from target) */
    private static final double RPM_READY_TOLERANCE = 1500.0;
    /** Minimum time (seconds) the shooter must hold target RPM before reporting ready */
    private static final double MIN_STABLE_TIME_SECONDS = 0.1;
    
    // Temperature monitoring
    private static final double TEMP_WARNING_THRESHOLD = 70.0;
    private boolean hasWarnedHighTemp = false;
    private int tempCheckCounter = 0; // Only check temps every N cycles
    
    // Reusable array not safe for potential multi-threaded/command usage, switching to local
    // private final double[] interpolatedPowers = new double[2];

    /**
     * Configures both motors in coast mode so flywheels spin down naturally.
     */
    public ShooterSubsystem() {
        topMotor = new TalonFX(Constants.Shooter.TOP_MOTOR_ID);
        bottomMotor = new TalonFX(Constants.Shooter.BOTTOM_MOTOR_ID);
        topControl = new DutyCycleOut(0);
        bottomControl = new DutyCycleOut(0);
        topVelocityControl = new VelocityVoltage(0).withSlot(0);
        bottomVelocityControl = new VelocityVoltage(0).withSlot(0);
        
        configureMotor(topMotor, Constants.Shooter.TOP_MOTOR_INVERTED);
        configureMotor(bottomMotor, Constants.Shooter.BOTTOM_MOTOR_INVERTED);
    }
    
    private void configureMotor(TalonFX motor, boolean inverted) {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        // Velocity PID gains for RPM-based control (Slot 0)
        // kV = feedforward (volts per RPS), kP = proportional gain
        config.Slot0.kV = 0.12;  // ~12V / 100 RPS = 0.12 V per RPS
        config.Slot0.kP = 0.11;  // Proportional correction
        config.Slot0.kI = 0.0;   // No integral for flywheel
        config.Slot0.kD = 0.0;   // No derivative for flywheel
        motor.getConfigurator().apply(config);
    }

    public boolean checkHealth() {
        boolean topMotorHealthy = topMotor.isAlive();
        boolean bottomMotorHealthy = bottomMotor.isAlive();
        isHealthy = topMotorHealthy && bottomMotorHealthy;
        return isHealthy;
    }

    public Boolean isHealthy() {
        return isHealthy;
    }

    // POWER CONTROL METHODS
    
    /**
     * @deprecated Calibration is now unified in ShootingCalculator.
     * Use {@link #setTargetRPM(double, double)} instead.
     */
    @Deprecated
    public void setForDistance(double distanceMeters) {
        this.currentDistance = distanceMeters;
        // Legacy: just set a default power. Use setTargetRPM() via ShootingCalculator instead.
        targetTopPower = 0.5;
        targetBottomPower = 0.5;
        spinUpStartTime = System.currentTimeMillis();
    }
    
    /**
     * @deprecated Calibration is now unified in ShootingCalculator.
     * Use {@link #setTargetRPM(double, double)} instead.
     */
    @Deprecated
    public void setForDistanceTrench(double distanceMeters) {
        this.currentDistance = distanceMeters;
        // Legacy: just set a default power. Use setTargetRPM() via ShootingCalculator instead.
        targetTopPower = 0.5;
        targetBottomPower = 0.5;
        spinUpStartTime = System.currentTimeMillis();
    }
    
    /**
     * Manually set shooter motor powers (bypasses calibration tables).
     * Used by the calibration command for testing different powers.
     * 
     * @param topPower Power for top motor (0.0 to 1.0)
     * @param bottomPower Power for bottom motor (0.0 to 1.0)
     */
    public void setManualPower(double topPower, double bottomPower) {
        targetTopPower = clamp(topPower, Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        targetBottomPower = clamp(bottomPower, Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        useVelocityControl = false;
        spinUpStartTime = System.currentTimeMillis();
    }
    
    /**
     * Set shooter target RPM using closed-loop velocity control.
     * This is more accurate than duty cycle since it compensates for battery voltage
     * and load changes automatically via the motor controller's PID loop.
     * 
     * @param topRPM Target RPM for top motor (0 to ~6000)
     * @param bottomRPM Target RPM for bottom motor (0 to ~6000)
     */
    public void setTargetRPM(double topRPM, double bottomRPM) {
        // Convert RPM to RPS for TalonFX (it uses rotations per second)
        double newTopRPS = topRPM / 60.0;
        double newBottomRPS = bottomRPM / 60.0;
        
        // Only reset spin-up timer if the target changed significantly.
        // AutoShootCommand calls this every 20ms cycle with slightly varying values
        // from interpolation. If we reset the timer every call, the time gate never passes.
        double topChange = Math.abs(newTopRPS - targetTopRPS) * 60.0; // back to RPM for comparison
        double bottomChange = Math.abs(newBottomRPS - targetBottomRPS) * 60.0;
        if (topChange > 100.0 || bottomChange > 100.0) {
            spinUpStartTime = System.currentTimeMillis();
        }
        
        targetTopRPS = newTopRPS;
        targetBottomRPS = newBottomRPS;
        
        // Also set duty cycle equivalents for telemetry/fallback
        targetTopPower = clamp(topRPM / (Constants.Shooter.MOTOR_FREE_SPEED_RPS * 60.0), 0.0, 1.0);
        targetBottomPower = clamp(bottomRPM / (Constants.Shooter.MOTOR_FREE_SPEED_RPS * 60.0), 0.0, 1.0);
        
        useVelocityControl = true;
    }
    
    /**
     * @deprecated Calibration is now unified in ShootingCalculator.
     * Use {@link #setTargetRPM(double, double)} instead.
     */
    @Deprecated
    public void setForDistanceWithOffset(double distanceMeters, double topOffset, double bottomOffset) {
        this.currentDistance = distanceMeters;
        targetTopPower = clamp(0.5 + topOffset, Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        targetBottomPower = clamp(0.5 + bottomOffset, Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        spinUpStartTime = System.currentTimeMillis();
    }
    
    /**
     * @deprecated Calibration is now unified in ShootingCalculator.
     * Use {@link #setTargetRPM(double, double)} instead.
     */
    @Deprecated
    public void setForDistanceTrenchWithOffset(double distanceMeters, double topOffset, double bottomOffset) {
        this.currentDistance = distanceMeters;
        targetTopPower = clamp(0.5 + topOffset, Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        targetBottomPower = clamp(0.5 + bottomOffset, Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        spinUpStartTime = System.currentTimeMillis();
    }
    
    /**
     * Stop both shooter motors immediately.
     */
    public void stop() {
        targetTopPower = 0.0;
        targetBottomPower = 0.0;
        isSpunUp = false;
    }
    
    // STATUS METHODS
    
    /**
     * Check if the shooter is ready to fire.
     * Requires motors to be spinning AND spin-up time to have elapsed.
     * 
     * @return true if ready to fire
     */
    public boolean isReady() {
        return true;
    }
    
    /**
     * Get current top motor power setting.
     * @return Power value (0.0 to 1.0)
     */
    public double getTopPower() {
        return targetTopPower;
    }
    
    /**
     * Get current top motor power setting.
     * Alias for getTopPower() for calibration commands.
     * @return Power value (0.0 to 1.0)
     */
    public double getCurrentTopPower() {
        return targetTopPower;
    }
    
    /**
     * Get current bottom motor power setting.
     * @return Power value (0.0 to 1.0)
     */
    public double getBottomPower() {
        return targetBottomPower;
    }
    
    /**
     * Get current bottom motor power setting.
     * Alias for getBottomPower() for calibration commands.
     * @return Power value (0.0 to 1.0)
     */
    public double getCurrentBottomPower() {
        return targetBottomPower;
    }
    
    /**
     * Get the distance currently being used for power calculations.
     * @return Distance in meters
     */
    public double getCurrentDistance() {
        return currentDistance;
    }
    
    /**
     * Get the current top motor RPM.
     * @return RPM (rotations per minute)
     */
    public double getTopRPM() {
        return topMotor.getVelocity().getValueAsDouble() * 60.0; // RPS to RPM
    }
    
    /**
     * Get the current bottom motor RPM.
     * @return RPM (rotations per minute)
     */
    public double getBottomRPM() {
        return bottomMotor.getVelocity().getValueAsDouble() * 60.0; // RPS to RPM
    }
    
    // INTERPOLATION
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    // PERIODIC UPDATE
    
    @Override
    public void periodic() {
        // SAFETY: When robot is disabled, do NOT send any CAN frames.
        // Phoenix 6 blocks motor control while disabled, causing "Could not transmit" errors.
        if (DriverStation.isDisabled()) {
            isSpunUp = false;
            useVelocityControl = false;
            DashboardHelper.putNumber(Category.MATCH, "Shooter/TargetTopRPM", 0.0);
            DashboardHelper.putNumber(Category.MATCH, "Shooter/TargetBottomRPM", 0.0);
            DashboardHelper.putNumber(Category.MATCH, "Shooter/ActualTopRPM", 0.0);
            DashboardHelper.putNumber(Category.MATCH, "Shooter/ActualBottomRPM", 0.0);
            DashboardHelper.putBoolean(Category.MATCH, "Shooter/Ready", false);
            return;
        }
        
        // Calculate effective power
        double effectiveTopPower = targetTopPower;
        double effectiveBottomPower = targetBottomPower;
        
        // Apply motor outputs - use velocity control or duty cycle
        if (useVelocityControl && targetTopRPS > 0.1) {
            // Closed-loop velocity control: motor PID handles RPM recovery automatically
            topMotor.setControl(topVelocityControl.withVelocity(targetTopRPS));
            bottomMotor.setControl(bottomVelocityControl.withVelocity(targetBottomRPS));
        } else {
            // Open-loop duty cycle control (manual / calibration / disabled)
            topMotor.setControl(topControl.withOutput(effectiveTopPower));
            bottomMotor.setControl(bottomControl.withOutput(effectiveBottomPower));
        }
        
        // Check spin-up status: verify actual RPM is close to target
        // Time gate: must have had at least MIN_STABLE_TIME_SECONDS since last setTargetRPM call
        // RPM gate: both motors must be within RPM_READY_TOLERANCE of their targets
        double elapsedSeconds = (System.currentTimeMillis() - spinUpStartTime) / 1000.0;
        if (useVelocityControl && targetTopRPS > 0.1) {
            double actualTopRPM = topMotor.getVelocity().getValueAsDouble() * 60.0;
            double actualBottomRPM = bottomMotor.getVelocity().getValueAsDouble() * 60.0;
            double topError = Math.abs(actualTopRPM - targetTopRPS * 60.0);
            double bottomError = Math.abs(actualBottomRPM - targetBottomRPS * 60.0);
            isSpunUp = topError < RPM_READY_TOLERANCE && 
                       bottomError < RPM_READY_TOLERANCE;
        } else {
            // Duty cycle fallback: just use time-based check
            isSpunUp = true;
        }
        
        // Publish telemetry: target RPM and actual RPM from motor encoders
        double actualTopRPMTelemetry = topMotor.getVelocity().getValueAsDouble() * 60.0;
        double actualBottomRPMTelemetry = bottomMotor.getVelocity().getValueAsDouble() * 60.0;
        DashboardHelper.putNumber(Category.MATCH, "Shooter/TargetTopRPM", targetTopRPS * 60.0);
        DashboardHelper.putNumber(Category.MATCH, "Shooter/TargetBottomRPM", targetBottomRPS * 60.0);
        DashboardHelper.putNumber(Category.MATCH, "Shooter/ActualTopRPM", actualTopRPMTelemetry);
        DashboardHelper.putNumber(Category.MATCH, "Shooter/ActualBottomRPM", actualBottomRPMTelemetry);
        DashboardHelper.putNumber(Category.MATCH, "Shooter/TopError", Math.abs(actualTopRPMTelemetry - targetTopRPS * 60.0));
        DashboardHelper.putNumber(Category.MATCH, "Shooter/BottomError", Math.abs(actualBottomRPMTelemetry - targetBottomRPS * 60.0));
        DashboardHelper.putBoolean(Category.MATCH, "Shooter/Ready", isSpunUp);
        
        // Check motor temps every 25 cycles (~500ms) to reduce CAN traffic
        tempCheckCounter++;
        if (tempCheckCounter >= 25) {
            tempCheckCounter = 0;
            checkMotorTemperatures();
        }
    }
    
    /**
     * Warns via Elastic notification if motors are getting too hot.
     * Only called every ~500ms to minimize CAN bus traffic.
     */
    private void checkMotorTemperatures() {
        double topTemp = topMotor.getDeviceTemp().getValueAsDouble();
        double bottomTemp = bottomMotor.getDeviceTemp().getValueAsDouble();
        double maxTemp = Math.max(topTemp, bottomTemp);
        
        if (maxTemp >= TEMP_WARNING_THRESHOLD && !hasWarnedHighTemp) {
            hasWarnedHighTemp = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle("Shooter Motor Hot!")
                .withDescription("Motor at " + (int)maxTemp + "C - consider resting")
                .withDisplaySeconds(5.0));
        } else if (maxTemp < TEMP_WARNING_THRESHOLD - 10) {
            hasWarnedHighTemp = false;
        }
    }
}