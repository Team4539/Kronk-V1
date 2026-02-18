package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controls the intake mechanism with a single pivot motor and a roller motor.
 * Uses a CANcoder for absolute position feedback and a WPILib PID controller
 * that works directly in degrees -- no gear ratio math needed.
 */
public class IntakeSubsystem extends SubsystemBase {
    
    // Hardware
    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;
    private final CANcoder pivotCANcoder;
    
    // Control -- just duty cycle, PID runs in our code in degrees
    private final DutyCycleOut pivotControl;
    private final DutyCycleOut rollerControl;
    private final PIDController pivotPID;
    
    // State
    private double targetPivotAngle = Constants.Intake.IDLE_ANGLE_DEG;
    private double targetRollerSpeed = 0.0;
    private double currentPivotAngle = 0.0;
    private boolean isDeployed = false;
    private boolean isIdle = true;
    private boolean isHealthy = true;

    public IntakeSubsystem() {
        pivotMotor = new TalonFX(Constants.Intake.RIGHT_PIVOT_MOTOR_ID);
        rollerMotor = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);
        pivotCANcoder = new CANcoder(Constants.Intake.RIGHT_CANCODER_ID);
        
        pivotControl = new DutyCycleOut(0);
        rollerControl = new DutyCycleOut(0);
        
        // PID works in degrees: error in degrees -> output in duty cycle
        pivotPID = new PIDController(
            Constants.Intake.PIVOT_PID_P,
            Constants.Intake.PIVOT_PID_I,
            Constants.Intake.PIVOT_PID_D
        );
        pivotPID.setTolerance(Constants.Intake.PIVOT_TOLERANCE_DEG);
        
        configureCANcoder();
        configurePivotMotor();
        configureRollerMotor();
        
        // Read starting position
        pivotCANcoder.getAbsolutePosition().waitForUpdate(0.1);
        currentPivotAngle = pivotCANcoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        System.out.println("[Intake] Starting angle: " + currentPivotAngle + " deg");
        
        retract();
    }
    
    private void configureCANcoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Constants.Intake.RIGHT_CANCODER_OFFSET_DEG / 360.0;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotCANcoder.getConfigurator().apply(config);
    }
    
    private void configurePivotMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = Constants.Intake.RIGHT_PIVOT_MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        pivotMotor.getConfigurator().apply(config);
    }
    
    private void configureRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = Constants.Intake.ROLLER_MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
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
        targetRollerSpeed = Constants.Intake.INTAKE_SPEED;
    }
    
    public void startOuttake() {
        targetRollerSpeed = Constants.Intake.OUTTAKE_SPEED;
    }
    
    public void stopRollers() {
        targetRollerSpeed = 0.0;
    }
    
    public void setRollerSpeed(double speed) {
        targetRollerSpeed = clamp(speed, -1.0, 1.0);
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
        return Math.abs(currentPivotAngle - targetPivotAngle) < Constants.Intake.PIVOT_TOLERANCE_DEG;
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
        return targetRollerSpeed;
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
        // 1. Read current angle directly from CANcoder in degrees
        currentPivotAngle = pivotCANcoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        
        // SAFETY: When robot is disabled, send zero to all motors.
        // No PID, no roller - just read sensors for telemetry.
        if (DriverStation.isDisabled()) {
            pivotMotor.setControl(pivotControl.withOutput(0));
            rollerMotor.setControl(rollerControl.withOutput(0));
            updateDashboard();
            return;
        }
        
        // 2. PID: current degrees -> target degrees -> duty cycle output
        double pidOutput = pivotPID.calculate(currentPivotAngle, targetPivotAngle);
        
        // 3. Clamp so it doesn't go full send
        pidOutput = clamp(pidOutput, -Constants.Intake.PIVOT_MAX_OUTPUT, Constants.Intake.PIVOT_MAX_OUTPUT);
        
        // 4. If at target, just stop -- don't fight with tiny oscillations
        if (pivotPID.atSetpoint()) {
            pidOutput = 0.0;
        }
        
        // 5. Send to motor
        pivotMotor.setControl(pivotControl.withOutput(pidOutput));
        
        // Roller
        rollerMotor.setControl(rollerControl.withOutput(targetRollerSpeed));
        
        // Dashboard
        updateDashboard();
    }
}
