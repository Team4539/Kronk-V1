package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controls the intake mechanism with a pivot motor, a roller motor,
 * and a CANcoder for absolute position feedback.
 * 
 * Pivot uses Phoenix 6 Motion Magic Position (MotionMagicVoltage) with the
 * CANcoder fused as a remote sensor. The Talon runs the PID loop on-board
 * at 1kHz for smooth, profiled motion through an 81:1 gear reduction.
 * 
 * Roller uses Motion Magic Velocity for consistent RPM control.
 */
public class IntakeSubsystem extends SubsystemBase {
    
    // Hardware
    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;
    private final CANcoder pivotCANcoder;
    
    // Control
    private final MotionMagicVoltage pivotMotionMagic;
    private final DutyCycleOut rollerDutyControl;
    private final MotionMagicVelocityVoltage rollerMotionMagic;
    
    // Separate Motion Magic configs for deploy vs retract
    // We pre-build both profiles and only re-apply when direction truly changes.
    private final MotionMagicConfigs deployProfile = new MotionMagicConfigs();
    private final MotionMagicConfigs retractProfile = new MotionMagicConfigs();
    private boolean lastDirectionWasDeploying = false;
    private boolean motionProfileDirty = true; // Force first apply
    private double lastTargetAngle = Double.NaN; // Track target changes
    
    // State
    private double targetPivotAngle = Constants.Intake.IDLE_ANGLE_DEG;
    private double targetRollerRPS = 0.0;
    private double currentPivotAngle = 0.0;
    private boolean isDeployed = false;
    private boolean isIdle = true;
    private boolean isHealthy = true;

    public IntakeSubsystem() {
        pivotMotor = new TalonFX(Constants.Intake.PIVOT_MOTOR_ID);
        rollerMotor = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);
        pivotCANcoder = new CANcoder(Constants.Intake.CANCODER_ID);
        
        // Pivot: Motion Magic Position (Slot 0 on pivot motor)
        pivotMotionMagic = new MotionMagicVoltage(0).withSlot(0);
        
        // Roller: Motion Magic Velocity (Slot 0 on roller motor)
        rollerDutyControl = new DutyCycleOut(0);
        rollerMotionMagic = new MotionMagicVelocityVoltage(0).withSlot(0);
        
        configureCANcoder();
        configurePivotMotor();
        configureRollerMotor();
        initMotionProfiles();
        
        // Read starting position from CANcoder
        pivotCANcoder.getAbsolutePosition().waitForUpdate(0.1);
        currentPivotAngle = pivotCANcoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        System.out.println("[Intake] Starting angle: " + currentPivotAngle + " deg");
        
        retract();
    }
    
    private void configureCANcoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Constants.Intake.CANCODER_OFFSET_DEG / 360.0;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotCANcoder.getConfigurator().apply(config);
    }
    
    private void configurePivotMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = Constants.Intake.PIVOT_MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        
        // Fuse CANcoder as remote sensor for absolute position
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = Constants.Intake.CANCODER_ID;
        config.Feedback.RotorToSensorRatio = Constants.Intake.PIVOT_GEAR_RATIO;  // 81:1
        config.Feedback.SensorToMechanismRatio = 1.0;  // CANcoder is 1:1 with output
        
        // Slot 0: DEPLOY — aggressive PID for fast, forceful deployment (going down)
        // Chain slop doesn't matter as much deploying because gravity helps.
        config.Slot0.kP = 45.0;   // High P for snappy deploy
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.3;    // Light damping — we want speed
        config.Slot0.kS = 0.15;   // Static friction feedforward
        config.Slot0.kG = 0.3;    // Gravity feedforward (pivot fights gravity)
        config.Slot0.kV = 0.0;    // Not needed for position control
        
        // Slot 1: RETRACT/IDLE — gentler PID to avoid overshooting through chain slop
        // Lower P reduces overshoot; moderate D damps oscillation without amplifying noise.
        config.Slot1.kP = 19.0;   // Lower P — approach gently
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 0.5;    // Moderate D — enough to damp, not amplify chain noise
        config.Slot1.kS = 0.15;
        config.Slot1.kG = 0.3;
        config.Slot1.kV = 0.0;
        
        // Motion Magic: default to the deploy (fast) profile.
        // We dynamically switch cruise/accel/jerk in periodic() based on direction.
        config.MotionMagic.MotionMagicCruiseVelocity = 0.5;  // rot/s at the output (~180 deg/s)
        config.MotionMagic.MotionMagicAcceleration = 1.0;     // rot/s² — reach cruise in 0.5s
        config.MotionMagic.MotionMagicJerk = 10.0;            // rot/s³ — S-curve smoothing
        
        // Software position limits (in mechanism rotations = degrees / 360)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.MAX_PIVOT_ANGLE_DEG / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Intake.MIN_PIVOT_ANGLE_DEG / 360.0;
        
        pivotMotor.getConfigurator().apply(config);
    }
    
    private void configureRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = Constants.Intake.ROLLER_MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        
        // Slot 0: velocity PID gains for Motion Magic Velocity
        config.Slot0.kV = 0.12;
        config.Slot0.kP = 0.10;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.05;
        
        // Motion Magic: acceleration & jerk limits for smooth roller control
        config.MotionMagic.MotionMagicAcceleration = 300;  // rps/s
        config.MotionMagic.MotionMagicJerk = 3000;         // rps/s²
        
        rollerMotor.getConfigurator().apply(config);
    }
    
    /**
     * Pre-build the two Motion Magic profiles so we never allocate in periodic().
     * We only re-apply a profile to the Talon when the direction actually changes.
     */
    private void initMotionProfiles() {
        // DEPLOY: fast and aggressive — slam it down
        deployProfile.MotionMagicCruiseVelocity = 0.6;  // rot/s (~216 deg/s)
        deployProfile.MotionMagicAcceleration = 1.5;     // rot/s²
        deployProfile.MotionMagicJerk = 15.0;            // rot/s³
        
        // RETRACT/IDLE: slow, gentle approach to avoid chain slop overshoot
        retractProfile.MotionMagicCruiseVelocity = 0.3;  // rot/s (~108 deg/s)
        retractProfile.MotionMagicAcceleration = 0.5;     // rot/s²
        retractProfile.MotionMagicJerk = 5.0;             // rot/s³
    }

    // Health

    public boolean checkHealth() {
        boolean pivotHealthy = pivotMotor.isAlive();
        boolean rollerHealthy = rollerMotor.isAlive();
        boolean cancoderHealthy = pivotCANcoder.isConnected();
        isHealthy = pivotHealthy && rollerHealthy && cancoderHealthy;
        return isHealthy;
    }

    public boolean isHealthy() {
        return isHealthy;
    }

    // Pivot control

    public void deploy() {
        targetPivotAngle = clamp(Constants.Intake.DEPLOYED_ANGLE_DEG,
                                 Constants.Intake.MIN_PIVOT_ANGLE_DEG,
                                 Constants.Intake.MAX_PIVOT_ANGLE_DEG);
        isDeployed = true;
        isIdle = false;
    }
    
    public void retract() {
        targetPivotAngle = clamp(Constants.Intake.IDLE_ANGLE_DEG,
                                 Constants.Intake.MIN_PIVOT_ANGLE_DEG,
                                 Constants.Intake.MAX_PIVOT_ANGLE_DEG);
        isDeployed = false;
        isIdle = true;
    }
    
    public void setPivotAngle(double angleDegrees) {
        targetPivotAngle = clamp(angleDegrees, 
                                 Constants.Intake.MIN_PIVOT_ANGLE_DEG, 
                                 Constants.Intake.MAX_PIVOT_ANGLE_DEG);
    }

    // Roller control

    public void startIntake() {
        targetRollerRPS = Constants.Intake.INTAKE_SPEED_RPM / 60.0;
    }
    
    public void startOuttake() {
        targetRollerRPS = Constants.Intake.OUTTAKE_SPEED_RPM / 60.0;
    }
    
    public void stopRollers() {
        targetRollerRPS = 0.0;
    }
    
    /**
     * Set roller to a custom speed.
     * @param rpm Target RPM (negative = reverse/outtake)
     */
    public void setRollerSpeed(double rpm) {
        targetRollerRPS = rpm / 60.0;
    }

    /**
     * Get the roller motor's current velocity in rotations per second.
     * Used for stall detection.
     */
    public double getRollerVelocityRPS() {
        return rollerMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Get the roller motor's supply current in amps.
     * High current + low velocity = stall condition.
     */
    public double getRollerSupplyCurrent() {
        return rollerMotor.getSupplyCurrent().getValueAsDouble();
    }

    // Combined control

    public void deployAndIntake() {
        deploy();
        startIntake();
    }
    
    public void stopAndRetract() {
        stopRollers();
        retract();
    }

    // Status

    public boolean isDeployed() {
        return isDeployed;
    }

    public String getIntakeState() {
        if (isDeployed) return "Deployed";
        if (isIdle) return "Idle";
        return "Retracted";
    }
    
    public boolean isPivotAtTarget() {
        // Closed-loop error is in mechanism rotations; convert to degrees
        double errorDeg = Math.abs(pivotMotor.getClosedLoopError().getValueAsDouble() * 360.0);
        return errorDeg < Constants.Intake.PIVOT_TOLERANCE_DEG;
    }
    
    public double getPivotAngle() {
        return currentPivotAngle;
    }
    
    public double getCurrentPivotAngle() {
        return currentPivotAngle;
    }
    
    public double getTargetPivotAngle() {
        return targetPivotAngle;
    }
    
    public double getCurrentRollerSpeed() {
        return rollerMotor.get();
    }
    
    public double getTargetRollerSpeed() {
        return targetRollerRPS * 60.0;
    }

    // Utility

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    private void updateDashboard() {
        DashboardHelper.putBoolean(Category.MATCH, "Intake/Deployed", isDeployed);
        DashboardHelper.putNumber(Category.DEBUG, "Intake/PivotAngle", currentPivotAngle);
        DashboardHelper.putNumber(Category.DEBUG, "Intake/PivotError", targetPivotAngle - currentPivotAngle);
    }

    // Periodic

    @Override
    public void periodic() {
        // 1. Read current angle from the fused CANcoder (mechanism rotations → degrees)
        currentPivotAngle = pivotMotor.getPosition().getValueAsDouble() * 360.0;
        
        // SAFETY: When robot is disabled, explicitly command zero output to both motors.
        // Just returning without a setControl() leaves the last command active.
        if (DriverStation.isDisabled()) {
            rollerMotor.setControl(rollerDutyControl.withOutput(0));
            pivotMotor.setControl(pivotMotionMagic.withPosition(currentPivotAngle / 360.0).withSlot(1));
            targetRollerRPS = 0.0;
            updateDashboard();
            return;
        }
        
        // 2. Pivot: Motion Magic Position — target in mechanism rotations (degrees / 360)
        //    Pick aggressive (Slot 0) for deploying, gentle (Slot 1) for retracting/idle.
        //    Only re-apply the Motion Magic profile when the target changes direction,
        //    NOT every cycle — re-applying mid-move resets the profile and causes shaking.
        double targetRotations = targetPivotAngle / 360.0;
        
        // Determine direction only when the target angle changes significantly.
        // Small changes (e.g., jiggle oscillation) skip direction re-evaluation to avoid
        // re-applying Motion Magic configs every cycle, which resets the profile and shakes.
        boolean targetChanged = Double.isNaN(lastTargetAngle) 
                || Math.abs(lastTargetAngle - targetPivotAngle) > 2.0;
        if (targetChanged || motionProfileDirty) {
            boolean deploying = (targetPivotAngle > currentPivotAngle + 5.0); // 5° hysteresis
            
            if (deploying != lastDirectionWasDeploying || motionProfileDirty) {
                lastDirectionWasDeploying = deploying;
                pivotMotor.getConfigurator().apply(deploying ? deployProfile : retractProfile);
            }
            lastTargetAngle = targetPivotAngle;
            motionProfileDirty = false;
        }
        
        int slot = lastDirectionWasDeploying ? 0 : 1;
        pivotMotor.setControl(pivotMotionMagic.withPosition(targetRotations).withSlot(slot));
        
        // 3. Roller: Motion Magic Velocity for non-zero, duty cycle 0 for stop
        if (Math.abs(targetRollerRPS) > 0.1) {
            rollerMotor.setControl(rollerMotionMagic.withVelocity(targetRollerRPS));
        } else {
            rollerMotor.setControl(rollerDutyControl.withOutput(0));
        }
        
        // Dashboard
        updateDashboard();
    }
}
