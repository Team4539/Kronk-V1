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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameStateManager;

/**
 * Turret subsystem - 270° rotation range with PID position control.
 */
public class TurretSubsystem extends SubsystemBase {
    
    private final TalonFX motor;
    private final PositionDutyCycle positionControl;
    private final GameStateManager gameState = GameStateManager.getInstance();
    
    private double currentAbsoluteAngle = 0.0;
    private double targetAbsoluteAngle = 0.0;
    private double commandedAngle = 0.0;
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

    /** Set target angle (-180 to +180). Calculates best path within limits. */
    public void setTargetAngle(double angle) {
        double normalizedTarget = normalizeAngle(angle);
        double option1 = calculateAbsoluteTarget(normalizedTarget, currentAbsoluteAngle);
        double option2 = option1 + (option1 > currentAbsoluteAngle ? -360 : 360);
        
        boolean option1Valid = isWithinLimits(option1);
        boolean option2Valid = isWithinLimits(option2);
        
        double bestTarget;
        if (option1Valid && option2Valid) {
            double travel1 = Math.abs(option1 - currentAbsoluteAngle);
            double travel2 = Math.abs(option2 - currentAbsoluteAngle);
            bestTarget = (travel1 <= travel2) ? option1 : option2;
        } else if (option1Valid) {
            bestTarget = option1;
        } else if (option2Valid) {
            bestTarget = option2;
        } else {
            bestTarget = clamp(option1, Constants.Turret.MIN_ANGLE_DEG, Constants.Turret.MAX_ANGLE_DEG);
        }
        
        targetAbsoluteAngle = bestTarget;
    }

    public double getCurrentAngle() { return normalizeAngle(currentAbsoluteAngle); }
    public double getCurrentAbsoluteAngle() { return currentAbsoluteAngle; }
    public double getTargetAngle() { return normalizeAngle(targetAbsoluteAngle); }
    public double getTargetAbsoluteAngle() { return targetAbsoluteAngle; }
    public double getMotorRotations() { return motor.getPosition().getValueAsDouble(); }
    
    public boolean isOnTarget() {
        return Math.abs(currentAbsoluteAngle - targetAbsoluteAngle) < Constants.Turret.ON_TARGET_TOLERANCE_DEG;
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
            commandedAngle = currentAbsoluteAngle;
            targetAbsoluteAngle = currentAbsoluteAngle;
            initialized = true;
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
            DashboardHelper.putNumber(Category.DEBUG, "Turret/AbsAngle", currentAbsoluteAngle);
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
                .withDescription("At " + (int)currentAbsoluteAngle + " deg")
                .withDisplaySeconds(2.0));
        } else if (!nearLimit) {
            hasWarnedNearLimit = false;
        }
    }

    private void updateCurrentAngle() {
        double motorRotations = motor.getPosition().getValueAsDouble();
        double turretRotations = motorRotations / Constants.Turret.GEAR_RATIO;
        currentAbsoluteAngle = turretRotations * 360.0;
        if (Constants.Turret.MOTOR_INVERTED) currentAbsoluteAngle = -currentAbsoluteAngle;
    }

    private void updateMotorPosition() {
        double motorRots = (commandedAngle / 360.0) * Constants.Turret.GEAR_RATIO;
        if (Constants.Turret.MOTOR_INVERTED) motorRots = -motorRots;
        motor.setControl(positionControl.withPosition(motorRots));
    }

    private void smoothCommandedAngle(double speedMultiplier) {
        long now = System.nanoTime();
        double dt = (now - lastTimestampNanos) / 1e9;
        if (dt <= 0 || dt > 1.0) dt = 0.02;
        lastTimestampNanos = now;

        double maxDelta = Constants.Turret.MAX_ANGULAR_SPEED_DEG_PER_SEC * dt * speedMultiplier;
        double delta = targetAbsoluteAngle - commandedAngle;
        
        commandedAngle = (Math.abs(delta) <= maxDelta) 
            ? targetAbsoluteAngle 
            : commandedAngle + Math.signum(delta) * maxDelta;
    }

    private void initializeOffsets() {
        for (int tagId : Constants.Turret.TAG_ANGLE_OFFSETS.keySet()) {
            DashboardHelper.putNumber(Category.SETTINGS, "Offset/Angle_Tag" + tagId, 
                Constants.Turret.TAG_ANGLE_OFFSETS.get(tagId));
            DashboardHelper.putNumber(Category.SETTINGS, "Offset/Distance_Tag" + tagId, 
                Constants.Turret.TAG_DISTANCE_OFFSETS.get(tagId));
        }
    }

    private double normalizeAngle(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    private double calculateAbsoluteTarget(double normalizedTarget, double currentAbsolute) {
        double diff = normalizedTarget - normalizeAngle(currentAbsolute);
        return currentAbsolute + normalizeAngle(diff);
    }
    
    private boolean isWithinLimits(double absoluteAngle) {
        return absoluteAngle >= Constants.Turret.MIN_ANGLE_DEG && absoluteAngle <= Constants.Turret.MAX_ANGLE_DEG;
    }
    
    public boolean isAtLimit() {
        double margin = 10.0;
        return currentAbsoluteAngle <= Constants.Turret.MIN_ANGLE_DEG + margin
            || currentAbsoluteAngle >= Constants.Turret.MAX_ANGLE_DEG - margin;
    }
    
    public void resetPosition() {
        motor.setPosition(0);
        currentAbsoluteAngle = 0;
        commandedAngle = 0;
        targetAbsoluteAngle = 0;
    }
}