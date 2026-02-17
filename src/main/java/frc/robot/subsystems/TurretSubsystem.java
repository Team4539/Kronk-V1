package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
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

/**
 * Turret subsystem with bounded 0-360 deg internal position tracking.
 * 
 * Internal position is bounded [0, 360] and never counts beyond. The turret starts
 * at STARTUP_ANGLE_DEG (180 deg) which represents "forward" (0 deg external).
 * 
 * There is a physical dead zone between MAX_ANGLE_DEG and MIN_ANGLE_DEG (going through
 * 360/0). If the target is on the other side of the dead zone, the turret takes the
 * long way around through the usable range to avoid the dead zone.
 * 
 * External interface: setTargetAngle() accepts -180 to +180 (0 = forward).
 *                     getCurrentAngle() returns -180 to +180 (0 = forward).
 */
public class TurretSubsystem extends SubsystemBase {
    
    private final TalonFX motor;
    private final PositionDutyCycle positionControl;
    private final GameStateManager gameState = GameStateManager.getInstance();
    
    /** Internal position in bounded 0-360 degrees. 180 = forward at startup. */
    private double currentAngle = Constants.Turret.STARTUP_ANGLE_DEG;
    /** Target in bounded 0-360 degrees. */
    private double targetAngle = Constants.Turret.STARTUP_ANGLE_DEG;
    /** Smoothed commanded angle in bounded 0-360 degrees. */
    private double commandedAngle = Constants.Turret.STARTUP_ANGLE_DEG;
    
    private long lastTimestampNanos = System.nanoTime();
    private boolean initialized = false;
    private boolean hasWarnedNearLimit = false;
    private int telemetryCounter = 0;

    public TurretSubsystem() {
        motor = new TalonFX(Constants.Turret.MOTOR_ID);
        positionControl = new PositionDutyCycle(0);
        
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.PeakForwardDutyCycle = Constants.Turret.MAX_OUTPUT;
        config.MotorOutput.PeakReverseDutyCycle = -Constants.Turret.MAX_OUTPUT;
        
        var pidConfig = new Slot0Configs();
        pidConfig.kP = Constants.Turret.PID_P;
        pidConfig.kI = Constants.Turret.PID_I;
        pidConfig.kD = Constants.Turret.PID_D;
        config.Slot0 = pidConfig;
        
        motor.getConfigurator().apply(config);
        initializeOffsets();
        lastTimestampNanos = System.nanoTime();
    }

    /** 
     * Set target angle in external coordinates (-180 to +180, 0 = forward).
     * Converts to internal 0-360 and routes around the dead zone if needed.
     */
    public void setTargetAngle(double externalAngle) {
        // Convert external angle (-180..+180) to internal (0..360)
        double internalTarget = externalToInternal(externalAngle);
        
        // Clamp to usable range
        internalTarget = clamp(internalTarget, Constants.Turret.MIN_ANGLE_DEG, Constants.Turret.MAX_ANGLE_DEG);
        
        // Check if the shortest path crosses the dead zone.
        // If so, go the long way around through the usable range.
        targetAngle = routeAroundDeadZone(currentAngle, internalTarget);
    }

    /** Get current angle in external coordinates (-180 to +180, 0 = forward). */
    public double getCurrentAngle() { return internalToExternal(currentAngle); }
    
    /** Get current angle in internal coordinates (0-360). */
    public double getCurrentAbsoluteAngle() { return currentAngle; }
    
    /** Get target angle in external coordinates (-180 to +180, 0 = forward). */
    public double getTargetAngle() { return internalToExternal(targetAngle); }
    
    /** Get target angle in internal coordinates (0-360). */
    public double getTargetAbsoluteAngle() { return targetAngle; }
    
    public double getMotorRotations() { return motor.getPosition().getValueAsDouble(); }
    
    public boolean isOnTarget() {
        return Math.abs(currentAngle - targetAngle) < Constants.Turret.ON_TARGET_TOLERANCE_DEG;
    }

    /** Get angle offset for a specific AprilTag. */
    public double getAngleOffset(int tagId) {
        return DashboardHelper.getNumber(Category.SETTINGS, "Offset/Angle_Tag" + tagId, 
            Constants.Turret.TAG_ANGLE_OFFSETS.getOrDefault(tagId, 0.0));
    }

    /** Get distance offset for a specific AprilTag. */
    public double getDistanceOffset(int tagId) {
        return DashboardHelper.getNumber(Category.SETTINGS, "Offset/Distance_Tag" + tagId, 
            Constants.Turret.TAG_DISTANCE_OFFSETS.getOrDefault(tagId, 0.0));
    }

    public void updatePIDGains(double p, double i, double d) {
        Slot0Configs pidConfig = new Slot0Configs();
        pidConfig.kP = p;
        pidConfig.kI = i;
        pidConfig.kD = d;
        motor.getConfigurator().apply(pidConfig);
    }

    @Override
    public void periodic() {
        updateCurrentAngle();
        
        if (!initialized) {
            commandedAngle = currentAngle;
            targetAngle = currentAngle;
            initialized = true;
        }
        
        // SAFETY: When robot is disabled, do NOT command any motor outputs.
        // Brake mode will hold position passively without drawing current.
        if (DriverStation.isDisabled()) {
            telemetryCounter++;
            DashboardHelper.putNumber(Category.TELEOP, "Turret/Angle", getCurrentAngle());
            DashboardHelper.putBoolean(Category.TELEOP, "Turret/OnTarget", isOnTarget());
            return;
        }
        
        if (gameState.isManualTurretControl()) {
            setTargetAngle(gameState.getManualTurretAngle());
        }
        
        double speedLimit = gameState.isPracticeSlowMotion() ? 0.5 : 1.0;
        smoothCommandedAngle(speedLimit);
        
        if (gameState.shouldTurretRun()) {
            updateMotorPosition();
        } else {
            motor.setControl(positionControl.withPosition(motor.getPosition().getValueAsDouble()));
        }
        
        telemetryCounter++;
        DashboardHelper.putNumber(Category.TELEOP, "Turret/Angle", getCurrentAngle());
        DashboardHelper.putBoolean(Category.TELEOP, "Turret/OnTarget", isOnTarget());
        
        if (telemetryCounter >= 10) {
            telemetryCounter = 0;
            DashboardHelper.putNumber(Category.DEBUG, "Turret/InternalAngle", currentAngle);
            checkLimitWarnings();
        }
    }
    
    private void checkLimitWarnings() {
        boolean nearLimit = isAtLimit();
        if (nearLimit && !hasWarnedNearLimit) {
            hasWarnedNearLimit = true;
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(NotificationLevel.WARNING)
                .withTitle("Turret Near Limit!")
                .withDescription("At " + (int)currentAngle + " deg internal (" + (int)getCurrentAngle() + " deg)")
                .withDisplaySeconds(2.0));
        } else if (!nearLimit) {
            hasWarnedNearLimit = false;
        }
    }

    private void updateCurrentAngle() {
        double motorRotations = motor.getPosition().getValueAsDouble();
        double turretRotations = motorRotations / Constants.Turret.GEAR_RATIO;
        double rawAngle = turretRotations * 360.0;
        if (Constants.Turret.MOTOR_INVERTED) rawAngle = -rawAngle;
        // Motor position 0 = STARTUP_ANGLE internally
        currentAngle = clamp(Constants.Turret.STARTUP_ANGLE_DEG + rawAngle, 0.0, 360.0);
    }

    private void updateMotorPosition() {
        // Convert internal commanded angle back to motor rotations
        // Motor position 0 = STARTUP_ANGLE internally
        double turretDegrees = commandedAngle - Constants.Turret.STARTUP_ANGLE_DEG;
        if (Constants.Turret.MOTOR_INVERTED) turretDegrees = -turretDegrees;
        double motorRots = (turretDegrees / 360.0) * Constants.Turret.GEAR_RATIO;
        motor.setControl(positionControl.withPosition(motorRots));
    }

    private void smoothCommandedAngle(double speedMultiplier) {
        long now = System.nanoTime();
        double dt = (now - lastTimestampNanos) / 1e9;
        if (dt <= 0 || dt > 1.0) dt = 0.02;
        lastTimestampNanos = now;

        double maxDelta = Constants.Turret.MAX_ANGULAR_SPEED_DEG_PER_SEC * dt * speedMultiplier;
        double delta = targetAngle - commandedAngle;
        
        commandedAngle = (Math.abs(delta) <= maxDelta) 
            ? targetAngle 
            : commandedAngle + Math.signum(delta) * maxDelta;
        
        // Clamp to usable range for safety
        commandedAngle = clamp(commandedAngle, Constants.Turret.MIN_ANGLE_DEG, Constants.Turret.MAX_ANGLE_DEG);
    }

    private void initializeOffsets() {
        for (int tagId : Constants.Turret.TAG_ANGLE_OFFSETS.keySet()) {
            DashboardHelper.putNumber(Category.SETTINGS, "Offset/Angle_Tag" + tagId, 
                Constants.Turret.TAG_ANGLE_OFFSETS.get(tagId));
            DashboardHelper.putNumber(Category.SETTINGS, "Offset/Distance_Tag" + tagId, 
                Constants.Turret.TAG_DISTANCE_OFFSETS.get(tagId));
        }
    }

    // ===== Coordinate Conversion =====

    /** Convert external angle (-180..+180, 0=forward) to internal (0..360). */
    private double externalToInternal(double externalAngle) {
        double internal = Constants.Turret.STARTUP_ANGLE_DEG + externalAngle;
        // Wrap to 0-360
        internal = ((internal % 360.0) + 360.0) % 360.0;
        return internal;
    }

    /** Convert internal angle (0..360) to external (-180..+180, 0=forward). */
    private double internalToExternal(double internalAngle) {
        double external = internalAngle - Constants.Turret.STARTUP_ANGLE_DEG;
        // Normalize to -180..+180
        external = ((external + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return external;
    }

    // ===== Dead Zone Routing =====
    private double routeAroundDeadZone(double from, double to) {

        return to;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    public boolean isAtLimit() {
        double margin = Constants.Turret.LIMIT_SAFETY_MARGIN_DEG;
        return currentAngle <= Constants.Turret.MIN_ANGLE_DEG + margin
            || currentAngle >= Constants.Turret.MAX_ANGLE_DEG - margin;
    }
    
    public void resetPosition() {
        motor.setPosition(0);
        currentAngle = Constants.Turret.STARTUP_ANGLE_DEG;
        commandedAngle = Constants.Turret.STARTUP_ANGLE_DEG;
        targetAngle = Constants.Turret.STARTUP_ANGLE_DEG;
    }
}