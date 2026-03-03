package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
        
        // Slot 0: position PID gains (used by Motion Magic Position)
        // Units: output volts per rotation of error
        config.Slot0.kP = 60.0;   // Proportional — volts per rotation of error
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.5;    // Damping to prevent overshoot
        config.Slot0.kS = 0.15;   // Static friction feedforward
        config.Slot0.kG = 0.3;    // Gravity feedforward (pivot fights gravity)
        config.Slot0.kV = 0.0;    // Not needed for position control
        
        // Motion Magic: smooth profiled motion for the pivot
        // Units are in mechanism rotations (CANcoder rotations since SensorToMechanism = 1:1)
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
        
        // SAFETY: When robot is disabled, do NOT send any CAN frames.
        if (DriverStation.isDisabled()) {
            updateDashboard();
            return;
        }
        
        // 2. Pivot: Motion Magic Position — target in mechanism rotations (degrees / 360)
        double targetRotations = targetPivotAngle / 360.0;
        pivotMotor.setControl(pivotMotionMagic.withPosition(targetRotations));
        
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
