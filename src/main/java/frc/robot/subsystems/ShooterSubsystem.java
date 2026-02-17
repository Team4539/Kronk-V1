package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final frc.robot.CalibrationManager calibrationManager = frc.robot.CalibrationManager.getInstance();
    
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
    
    // RPM recovery compensation - auto-boosts power when balls sap flywheel speed
    private double topRpmBoost = 0.0;
    private double bottomRpmBoost = 0.0;
    
    // Reusable array not safe for potential multi-threaded/command usage, switching to local
    // private final double[] interpolatedPowers = new double[2];

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
     * Interpolates motor powers from calibration table. 
     */
    private double[] interpolatePowers(double distance, TreeMap<Double, double[]> calibration) {
        // Create local array to avoid concurrency issues
        double[] result = new double[2];

        // Get surrounding entries
        Map.Entry<Double, double[]> lower = calibration.floorEntry(distance);
        Map.Entry<Double, double[]> upper = calibration.ceilingEntry(distance);
        
        // Edge cases - use boundary values
        if (lower == null && upper == null) {
            result[0] = 0.5;
            result[1] = 0.5;
            return result;
        }
        if (lower == null) {
            double[] vals = upper.getValue();
            result[0] = vals[0];
            result[1] = vals[1];
            return result;
        }
        if (upper == null) {
            double[] vals = lower.getValue();
            result[0] = vals[0];
            result[1] = vals[1];
            return result;
        }
        
        // Linear interpolation
        double[] lo = lower.getValue();
        double[] hi = upper.getValue();
        double range = upper.getKey() - lower.getKey();
        if (range == 0) {
            // Same key -- just use the value
            result[0] = lo[0];
            result[1] = lo[1];
            return result;
        }
        double t = (distance - lower.getKey()) / range;
        result[0] = lo[0] + (hi[0] - lo[0]) * t;
        result[1] = lo[1] + (hi[1] - lo[1]) * t;
        return result;
    }
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    // RPM RECOVERY COMPENSATION
    
    /**
     * Calculates how much extra power to add when flywheel RPM drops below target.
     * This compensates for energy loss when balls hit the flywheels during rapid fire.
     * 
     * How it works:
     *   1. Expected RPM = targetPower x motor free speed
     *   2. Actual RPM = read from motor velocity sensor
     *   3. If actual < threshold% of expected -> boost proportionally to the deficit
     *   4. Boost is clamped to RPM_RECOVERY_MAX_BOOST to prevent runaway
     * 
     * @param targetPower The desired power (0.0 to 1.0)
     * @param actualVelocityRps The actual motor velocity in rotations per second
     * @return Extra power to add (0.0 if no recovery needed)
     */
    private double calculateRpmRecoveryBoost(double targetPower, double actualVelocityRps) {
        // Don't compensate if we're not trying to spin
        if (targetPower < 0.05) return 0.0;
        
        // Expected velocity based on target duty cycle
        double expectedRps = targetPower * Constants.Shooter.MOTOR_FREE_SPEED_RPS;
        
        // If actual RPM is below threshold of expected, calculate boost
        double rpmRatio = (expectedRps > 0) ? Math.abs(actualVelocityRps) / expectedRps : 1.0;
        
        if (rpmRatio < Constants.Shooter.RPM_RECOVERY_THRESHOLD) {
            // How far below target we are, as a fraction (e.g. 0.10 = 10% below)
            double deficit = 1.0 - rpmRatio;
            // Proportional boost, clamped to max
            return Math.min(deficit * Constants.Shooter.RPM_RECOVERY_GAIN, 
                           Constants.Shooter.RPM_RECOVERY_MAX_BOOST);
        }
        
        return 0.0;
    }
    
    // PERIODIC UPDATE
    
    @Override
    public void periodic() {
        // SAFETY: When robot is disabled, send zero power and skip all motor logic.
        // Coast mode means flywheels spin down naturally without drawing current.
        if (DriverStation.isDisabled()) {
            topMotor.setControl(topControl.withOutput(0));
            bottomMotor.setControl(bottomControl.withOutput(0));
            isSpunUp = false;
            DashboardHelper.putNumber(Category.TELEOP, "Shooter/TopPower", 0.0);
            DashboardHelper.putNumber(Category.TELEOP, "Shooter/BottomPower", 0.0);
            DashboardHelper.putBoolean(Category.TELEOP, "Shooter/Ready", false);
            return;
        }
        
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
        
        // CalibrationManager manual override (for tuning shot power)
        if (calibrationManager.useManualShooterPower()) {
            targetTopPower = calibrationManager.getShooterTopPower();
            targetBottomPower = calibrationManager.getShooterBottomPower();
        }
        
        // Calculate effective power
        double effectiveTopPower = shooterDisabled ? 0 : targetTopPower * speedLimit;
        double effectiveBottomPower = shooterDisabled ? 0 : targetBottomPower * speedLimit;
        
        // RPM recovery: read actual motor speed and boost power if RPM has dropped
        // (e.g. when balls sap flywheel energy during rapid fire)
        if (!shooterDisabled && effectiveTopPower > 0.05) {
            double topVelocityRps = topMotor.getVelocity().getValueAsDouble();
            double bottomVelocityRps = bottomMotor.getVelocity().getValueAsDouble();
            
            topRpmBoost = calculateRpmRecoveryBoost(effectiveTopPower, topVelocityRps);
            bottomRpmBoost = calculateRpmRecoveryBoost(effectiveBottomPower, bottomVelocityRps);
            
            effectiveTopPower = clamp(effectiveTopPower + topRpmBoost, 0, Constants.Shooter.MAX_POWER);
            effectiveBottomPower = clamp(effectiveBottomPower + bottomRpmBoost, 0, Constants.Shooter.MAX_POWER);
        } else {
            topRpmBoost = 0.0;
            bottomRpmBoost = 0.0;
        }
        
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
        DashboardHelper.putNumber(Category.TELEOP, "Shooter/RPM_Boost", Math.max(topRpmBoost, bottomRpmBoost));
        
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