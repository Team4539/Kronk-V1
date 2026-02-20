package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Turret subsystem using a TalonFX motor with PositionDutyCycle control.
 * 
 * COORDINATE SYSTEM:
 *   External angles: -180 to +180 degrees, where 0 = ROBOT FORWARD.
 *     +90 = left, -90 = right, ±180 = backward.
 *     Positive = CCW when viewed from above (standard math convention).
 *   Internal angles: 0 to 360 degrees, offset so power-on position is STARTUP_ANGLE_DEG.
 * 
 * MOTOR DIRECTION:
 *   MOTOR_INVERTED = true. The physical motor CW = positive encoder counts,
 *   but we need increasing internal angle = CCW (math convention).
 *   Phoenix 6 inversion flips both encoder and output together, so the math
 *   stays consistent: positive motor rotations = CCW turret movement.
 * 
 * HOME ANGLE:
 *   The turret physically faces LEFT (90° external) at power-on.
 *   HOME_ANGLE_DEG = 90 tells the conversion math about this offset.
 *   The conversions ensure external 0° always means "robot forward" regardless
 *   of where the turret physically starts.
 * 
 *   Example mappings (HOME=90, STARTUP=180):
 *     External  90° (left/home)    → Internal 180° (startup position, no motor movement)
 *     External   0° (forward)      → Internal  90° (motor turns CW from left to forward)
 *     External -90° (right)        → Internal   0°/360° (near dead zone)
 * 
 * DEAD ZONE:
 *   Usable range: Internal [75, 224] — contains 90 (forward) and 180 (home).
 *   Dead zone: Internal (224, 360) ∪ [0, 75) — turret cannot traverse this arc.
 *   External usable range: roughly -105° (right) through 0° (forward) to 134° (left/rear).
 * 
 * POSITION TRACKING:
 *   On startup, the motor encoder reads 0.0 rotations. We define this as STARTUP_ANGLE_DEG (180).
 *   currentInternalAngle = STARTUP_ANGLE_DEG + (motorRotations / GEAR_RATIO) * 360.0
 *   We then wrap to 0-360 to keep the internal representation bounded.
 */
public class TurretSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0.0);

    /** Current internal angle (0-360, bounded). Updated every cycle. */
    private double currentInternalAngle = Constants.Turret.STARTUP_ANGLE_DEG;

    /** Target internal angle the turret is moving toward. */
    private double targetInternalAngle = Constants.Turret.STARTUP_ANGLE_DEG;

    /** Whether we have a valid target set (false = hold current position). */
    private boolean hasTarget = false;

    public TurretSubsystem() {
        motor = new TalonFX(Constants.Turret.MOTOR_ID);

        // Configure motor
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor direction
        config.MotorOutput.Inverted = Constants.Turret.MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // PID (slot 0) - operates on motor rotations
        config.Slot0.kP = Constants.Turret.PID_P;
        config.Slot0.kI = Constants.Turret.PID_I;
        config.Slot0.kD = Constants.Turret.PID_D;

        // Peak output limit
        config.MotorOutput.PeakForwardDutyCycle = Constants.Turret.MAX_OUTPUT;
        config.MotorOutput.PeakReverseDutyCycle = -Constants.Turret.MAX_OUTPUT;

        motor.getConfigurator().apply(config);

        // Motor starts at position 0.0 rotations = STARTUP_ANGLE_DEG internally
        motor.setPosition(0.0);

        System.out.println("[Turret] Initialized | Internal startup=" + Constants.Turret.STARTUP_ANGLE_DEG
                + " | Range=[" + Constants.Turret.MIN_ANGLE_DEG + ", " + Constants.Turret.MAX_ANGLE_DEG + "]"
                + " | Gear ratio=" + Constants.Turret.GEAR_RATIO);
    }

    // ========================================================================
    // PERIODIC
    // ========================================================================

    @Override
    public void periodic() {
        // Update position from motor encoder
        updateCurrentAngle();

        // SAFETY: When robot is disabled, do NOT send any CAN frames.
        if (DriverStation.isDisabled()) {
            SmartDashboard.putNumber("Turret/Angle", getCurrentAngle());
            SmartDashboard.putNumber("Turret/InternalAngle", currentInternalAngle);
            SmartDashboard.putNumber("Turret/MotorRotations", motor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Turret/Target", getTargetExternalAngle());
            SmartDashboard.putBoolean("Turret/OnTarget", isOnTarget());
            return;
        }

        // Command motor if we have a target
        if (hasTarget) {
            // Convert target internal angle to motor rotations
            double targetRotations = internalAngleToMotorRotations(targetInternalAngle);
            motor.setControl(positionRequest.withPosition(targetRotations));
        }

        // Telemetry (just what matters)
        SmartDashboard.putNumber("Turret/Angle", getCurrentAngle());
        SmartDashboard.putNumber("Turret/InternalAngle", currentInternalAngle);
        SmartDashboard.putNumber("Turret/MotorRotations", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret/Target", getTargetExternalAngle());
        SmartDashboard.putBoolean("Turret/OnTarget", isOnTarget());
    }

    // ========================================================================
    // POSITION TRACKING
    // ========================================================================

    /**
     * Updates currentInternalAngle from the motor encoder.
     * Motor rotations are converted to degrees and offset from STARTUP_ANGLE_DEG.
     * Result is wrapped to [0, 360).
     */
    private void updateCurrentAngle() {
        double motorRotations = motor.getPosition().getValueAsDouble();
        double degreesFromStartup = (motorRotations / Constants.Turret.GEAR_RATIO) * 360.0;
        double raw = Constants.Turret.STARTUP_ANGLE_DEG + degreesFromStartup;

        // Wrap to [0, 360)
        currentInternalAngle = ((raw % 360.0) + 360.0) % 360.0;
    }

    // ========================================================================
    // ANGLE CONVERSIONS
    // ========================================================================

    /**
     * Convert external angle (-180 to +180, 0=forward) to internal angle (0-360).
     * 
     * At power-on: internal = STARTUP_ANGLE_DEG, external = HOME_ANGLE_DEG.
     * So: internal = external - HOME_ANGLE_DEG + STARTUP_ANGLE_DEG, wrapped to [0, 360).
     */
    private double externalToInternal(double externalDeg) {
        double internal = externalDeg - Constants.Turret.HOME_ANGLE_DEG + Constants.Turret.STARTUP_ANGLE_DEG;
        return ((internal % 360.0) + 360.0) % 360.0;
    }

    /**
     * Convert internal angle (0-360) to external angle (-180 to +180, 0=forward).
     * 
     * external = internal - STARTUP_ANGLE_DEG + HOME_ANGLE_DEG, normalized to [-180, +180).
     */
    private double internalToExternal(double internalDeg) {
        double external = internalDeg - Constants.Turret.STARTUP_ANGLE_DEG + Constants.Turret.HOME_ANGLE_DEG;
        // Normalize to [-180, +180)
        external = ((external + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return external;
    }

    /**
     * Convert internal angle to motor rotations (relative to startup position).
     * At startup, motor is at 0 rotations and internal angle is STARTUP_ANGLE_DEG.
     * 
     * We compute the shortest-path delta from STARTUP to target, then convert to rotations.
     * This handles wrapping correctly -- the motor position is unbounded (it can go negative).
     */
    private double internalAngleToMotorRotations(double targetInternal) {
        // Delta from startup position in degrees
        double delta = targetInternal - Constants.Turret.STARTUP_ANGLE_DEG;
        // Normalize to [-180, +180) to take shortest path
        delta = ((delta + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        // Convert degrees to motor rotations
        return (delta / 360.0) * Constants.Turret.GEAR_RATIO;
    }

    // ========================================================================
    // DEAD ZONE HANDLING
    // ========================================================================

    /**
     * Check if an internal angle is in the dead zone.
     * 
     * Usable range: [MIN_ANGLE_DEG (75), MAX_ANGLE_DEG (224)]
     *   This arc contains 90 (forward) and 180 (home/startup).
     * Dead zone: (224, 360) ∪ [0, 75) — the arc the turret cannot traverse.
     */
    private boolean isInDeadZone(double internalAngle) {
        return internalAngle > Constants.Turret.MAX_ANGLE_DEG
                || internalAngle < Constants.Turret.MIN_ANGLE_DEG;
    }

    /**
     * Clamp an internal angle to the nearest usable limit.
     * If the angle is in the dead zone, snap to whichever limit is closer.
     */
    private double clampToUsableRange(double internalAngle) {
        if (!isInDeadZone(internalAngle)) {
            return internalAngle; // Already in usable range
        }

        // Distance to each limit, going through the dead zone
        double distToMin = shortestAngularDistance(internalAngle, Constants.Turret.MIN_ANGLE_DEG);
        double distToMax = shortestAngularDistance(internalAngle, Constants.Turret.MAX_ANGLE_DEG);

        // Apply safety margin so we don't sit right on the edge
        if (distToMin <= distToMax) {
            return Constants.Turret.MIN_ANGLE_DEG + Constants.Turret.LIMIT_SAFETY_MARGIN_DEG;
        } else {
            return Constants.Turret.MAX_ANGLE_DEG - Constants.Turret.LIMIT_SAFETY_MARGIN_DEG;
        }
    }

    /**
     * Shortest angular distance between two angles (unsigned, 0-180).
     */
    private double shortestAngularDistance(double from, double to) {
        double diff = ((to - from) % 360.0 + 360.0) % 360.0;
        return diff > 180.0 ? 360.0 - diff : diff;
    }

    // ========================================================================
    // PUBLIC API
    // ========================================================================

    /**
     * Set the target turret angle in EXTERNAL coordinates.
     * 
     * @param externalAngleDeg Target angle in degrees, -180 to +180, where 0 = forward.
     *                         Positive = CCW (standard math convention).
     */
    public void setTargetAngle(double externalAngleDeg) {
        double internal = externalToInternal(externalAngleDeg);
        targetInternalAngle = clampToUsableRange(internal);
        hasTarget = true;
    }

    /**
     * Get the current turret angle in EXTERNAL coordinates.
     * 
     * @return Current angle in degrees, -180 to +180, where 0 = forward.
     */
    public double getCurrentAngle() {
        return internalToExternal(currentInternalAngle);
    }

    /**
     * Get the current target angle in external coordinates.
     * 
     * @return Target angle in degrees, -180 to +180, where 0 = forward.
     */
    public double getTargetExternalAngle() {
        return internalToExternal(targetInternalAngle);
    }

    /**
     * Check if the turret is within tolerance of the target angle.
     * 
     * @return true if error is less than ON_TARGET_TOLERANCE_DEG
     */
    public boolean isOnTarget() {
        if (!hasTarget) return false;
        double error = shortestAngularDistance(currentInternalAngle, targetInternalAngle);
        return error < Constants.Turret.ON_TARGET_TOLERANCE_DEG;
    }

    /**
     * Reset the turret position to assume it is currently facing forward.
     * Use after physically centering the turret.
     */
    public void resetPosition() {
        motor.setPosition(0.0);
        currentInternalAngle = Constants.Turret.STARTUP_ANGLE_DEG;
        targetInternalAngle = Constants.Turret.STARTUP_ANGLE_DEG;
        hasTarget = false;
        System.out.println("[Turret] Position reset to forward (internal=" + Constants.Turret.STARTUP_ANGLE_DEG + ")");
    }

    /**
     * Stop the turret motor and clear the target.
     */
    public void stop() {
        hasTarget = false;
        motor.stopMotor();
    }

    /**
     * Get the raw motor position in rotations (for calibration/debugging).
     */
    public double getMotorRotations() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Get the motor velocity in rotations per second (for calibration/debugging).
     */
    public double getMotorVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Update PID gains live on the motor (for calibration).
     * Applies new Slot0 PID values without full motor reconfiguration.
     * 
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     */
    public void updatePIDGains(double p, double i, double d) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Read current config first (preserves other settings)
        motor.getConfigurator().refresh(config);
        config.Slot0.kP = p;
        config.Slot0.kI = i;
        config.Slot0.kD = d;
        motor.getConfigurator().apply(config);
        System.out.println("[Turret] PID updated: P=" + p + " I=" + i + " D=" + d);
    }
}
