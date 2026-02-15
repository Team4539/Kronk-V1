package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameStateManager;

import java.util.Map;
import java.util.TreeMap;

/**
 * Dual flywheel shooter - launches game pieces with style!
 * 
 * Two motors work together:
 * - Top motor controls arc (backspin = higher trajectory)
 * - Bottom motor controls distance (more power = further)
 * 
 * Calibration tables map distance to power, interpolating for smooth curves.
 */
public class ShooterSubsystem extends SubsystemBase {
    
    // Hardware
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private final DutyCycleOut topControl;
    private final DutyCycleOut bottomControl;

    // Hardware Healthy
    public boolean isHealthy;

    // Cached reference to avoid repeated getInstance() calls
    private final GameStateManager gameState = GameStateManager.getInstance();
    
    // State
    private double targetTopPower = 0.0;
    private double targetBottomPower = 0.0;
    private double currentDistance = 0.0;
    private boolean isSpunUp = false;
    private long spinUpStartTime = 0;
    
    // Temperature monitoring
    private static final double TEMP_WARNING_THRESHOLD = 70.0;
    private boolean hasWarnedHighTemp = false;
    private int tempCheckCounter = 0; // Only check temps every N cycles
    
    // Reusable array to avoid allocations in interpolatePowers
    private final double[] interpolatedPowers = new double[2];

    /**
     * Sets up the shooter motors in coast mode (flywheels spin freely).
     */
    public ShooterSubsystem() {
        topMotor = new TalonFX(Constants.Shooter.TOP_MOTOR_ID);
        bottomMotor = new TalonFX(Constants.Shooter.BOTTOM_MOTOR_ID);
        topControl = new DutyCycleOut(0);
        bottomControl = new DutyCycleOut(0);
        
        configureMotor(topMotor, Constants.Shooter.TOP_MOTOR_INVERTED);
        configureMotor(bottomMotor, Constants.Shooter.BOTTOM_MOTOR_INVERTED);
    }
    
    private void configureMotor(TalonFX motor, boolean inverted) {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config);
    }

    public boolean checkHealth() {
        boolean topMotorHealthy = (topMotor.isAlive() && topMotor.getDeviceTemp() != null);
        boolean bottemMotorHealthy = (bottomMotor.isAlive() && bottomMotor.getDeviceTemp() != null);

        if (topMotorHealthy && bottemMotorHealthy){
            return isHealthy = true;
        }
        else 
        { 
            return isHealthy = false;
        }
    }

    public Boolean isHealthy() {
        return isHealthy();
    }

    // POWER CONTROL METHODS
    
    /**
     * Set shooter power based on distance to HUB target.
     * Uses the hub calibration table to determine appropriate powers.
     * 
     * @param distanceMeters Distance to hub in meters
     */
    public void setForDistance(double distanceMeters) {
        this.currentDistance = distanceMeters;
        
        // Look up and interpolate powers from calibration table
        double[] powers = interpolatePowers(distanceMeters, Constants.Shooter.SHOOTING_CALIBRATION);
        
        // Apply global offsets for fine-tuning
        targetTopPower = clamp(powers[0] + Constants.Shooter.TOP_MOTOR_POWER_OFFSET, 
                               Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        targetBottomPower = clamp(powers[1] + Constants.Shooter.BOTTOM_MOTOR_POWER_OFFSET, 
                                  Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        
        // Start spin-up timer
        spinUpStartTime = System.currentTimeMillis();
    }
    
    /**
     * Set shooter power based on distance to TRENCH target (for shuttling).
     * Uses the trench calibration table (typically flatter trajectory).
     * 
     * @param distanceMeters Distance to trench target in meters
     */
    public void setForDistanceTrench(double distanceMeters) {
        this.currentDistance = distanceMeters;
        
        // Look up and interpolate powers from trench calibration table
        double[] powers = interpolatePowers(distanceMeters, Constants.Shooter.TRENCH_CALIBRATION);
        
        // Apply global offsets
        targetTopPower = clamp(powers[0] + Constants.Shooter.TOP_MOTOR_POWER_OFFSET, 
                               Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        targetBottomPower = clamp(powers[1] + Constants.Shooter.BOTTOM_MOTOR_POWER_OFFSET, 
                                  Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        
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
        spinUpStartTime = System.currentTimeMillis();
    }
    
    /**
     * Set shooter power based on distance with additional offsets.
     * Used for calibration to test global offset values.
     * 
     * @param distanceMeters Distance to target in meters
     * @param topOffset Additional offset for top motor power
     * @param bottomOffset Additional offset for bottom motor power
     */
    public void setForDistanceWithOffset(double distanceMeters, double topOffset, double bottomOffset) {
        this.currentDistance = distanceMeters;
        
        // Look up and interpolate powers from calibration table
        double[] powers = interpolatePowers(distanceMeters, Constants.Shooter.SHOOTING_CALIBRATION);
        
        // Apply both global constants offsets AND additional calibration offsets
        targetTopPower = clamp(powers[0] + Constants.Shooter.TOP_MOTOR_POWER_OFFSET + topOffset, 
                               Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        targetBottomPower = clamp(powers[1] + Constants.Shooter.BOTTOM_MOTOR_POWER_OFFSET + bottomOffset, 
                                  Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        
        spinUpStartTime = System.currentTimeMillis();
    }
    
    /**
     * Set shooter power based on distance to TRENCH with additional offsets.
     * Uses the trench calibration table with training adjustments.
     * 
     * @param distanceMeters Distance to trench target in meters
     * @param topOffset Additional offset for top motor power (from training)
     * @param bottomOffset Additional offset for bottom motor power (from training)
     */
    public void setForDistanceTrenchWithOffset(double distanceMeters, double topOffset, double bottomOffset) {
        this.currentDistance = distanceMeters;
        
        // Look up and interpolate powers from TRENCH calibration table
        double[] powers = interpolatePowers(distanceMeters, Constants.Shooter.TRENCH_CALIBRATION);
        
        // Apply both global constants offsets AND additional calibration offsets
        targetTopPower = clamp(powers[0] + Constants.Shooter.TOP_MOTOR_POWER_OFFSET + topOffset, 
                               Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        targetBottomPower = clamp(powers[1] + Constants.Shooter.BOTTOM_MOTOR_POWER_OFFSET + bottomOffset, 
                                  Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        
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
        return isSpunUp && targetTopPower > 0.05;
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
    
    // INTERPOLATION
    
    /**
     * Interpolates motor powers from calibration table. Reuses internal array to avoid allocations.
     */
    private double[] interpolatePowers(double distance, TreeMap<Double, double[]> calibration) {
        // Get surrounding entries
        Map.Entry<Double, double[]> lower = calibration.floorEntry(distance);
        Map.Entry<Double, double[]> upper = calibration.ceilingEntry(distance);
        
        // Edge cases - use boundary values
        if (lower == null && upper == null) {
            interpolatedPowers[0] = 0.5;
            interpolatedPowers[1] = 0.5;
            return interpolatedPowers;
        }
        if (lower == null) {
            double[] vals = upper.getValue();
            interpolatedPowers[0] = vals[0];
            interpolatedPowers[1] = vals[1];
            return interpolatedPowers;
        }
        if (upper == null) {
            double[] vals = lower.getValue();
            interpolatedPowers[0] = vals[0];
            interpolatedPowers[1] = vals[1];
            return interpolatedPowers;
        }
        
        // Linear interpolation
        double t = (distance - lower.getKey()) / (upper.getKey() - lower.getKey());
        double[] lo = lower.getValue();
        double[] hi = upper.getValue();
        interpolatedPowers[0] = lo[0] + (hi[0] - lo[0]) * t;
        interpolatedPowers[1] = lo[1] + (hi[1] - lo[1]) * t;
        return interpolatedPowers;
    }
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    // PERIODIC UPDATE
    
    @Override
    public void periodic() {
        // Check if shooter should be disabled
        boolean shooterDisabled = !gameState.shouldShooterRun();
        
        // Get speed limit (cache to avoid multiple dashboard reads)
        double speedLimit = gameState.isPracticeSlowMotion() ? 0.5 : 
            DashboardHelper.getNumber(Category.PRACTICE, "ShooterSpeedLimit", 1.0);
        
        // Manual control overrides (only check in test mode)
        if (gameState.isManualShooterControl()) {
            double manualPower = gameState.getManualShooterRPM() / 6000.0;
            targetTopPower = clamp(manualPower * speedLimit, 0, 1.0);
            targetBottomPower = targetTopPower;
        }
        
        // Calculate effective power
        double effectiveTopPower = shooterDisabled ? 0 : targetTopPower * speedLimit;
        double effectiveBottomPower = shooterDisabled ? 0 : targetBottomPower * speedLimit;
        
        // Apply motor outputs
        topMotor.setControl(topControl.withOutput(effectiveTopPower));
        bottomMotor.setControl(bottomControl.withOutput(effectiveBottomPower));
        
        // Check spin-up status
        double elapsedSeconds = (System.currentTimeMillis() - spinUpStartTime) / 1000.0;
        isSpunUp = elapsedSeconds >= Constants.Shooter.SPIN_UP_TIME_SECONDS && effectiveTopPower > 0.05;
        
        // Publish essential telemetry only
        DashboardHelper.putNumber(Category.TELEOP, "Shooter/TopPower", effectiveTopPower);
        DashboardHelper.putNumber(Category.TELEOP, "Shooter/BottomPower", effectiveBottomPower);
        DashboardHelper.putBoolean(Category.TELEOP, "Shooter/Ready", isSpunUp);
        
        // Check motor temps every 25 cycles (~500ms) to reduce CAN traffic
        tempCheckCounter++;
        if (tempCheckCounter >= 25) {
            tempCheckCounter = 0;
            checkMotorTemperatures();
        }
    }
    
    /**
     * Warns if motors are getting too hot. Only called periodically to save CAN bandwidth.
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