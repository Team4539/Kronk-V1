package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    
    private final TalonFX motor;
    private final DutyCycleOut dutyControl;
    private final MotionMagicVelocityVoltage motionMagicVelocity;
    public boolean isHealthy;
    
    private double targetPower = 0.0;
    private double targetRPS = 0.0;
    private boolean useVelocityControl = false;
    private double currentDistance = 0.0;
    private boolean isSpunUp = false;
    private long spinUpStartTime = 0;
    
    private static final double RPM_READY_TOLERANCE = 300.0;
    private static final double TEMP_WARNING_THRESHOLD = 70.0;
    private boolean hasWarnedHighTemp = false;
    private int tempCheckCounter = 0;
    
    // Stall detection state machine
    private enum StallState { NORMAL, REVERSING }
    private StallState stallState = StallState.NORMAL;
    /** Timestamp when the RPM first dropped to ~0 while commanded to spin */
    private double stallStartTime = 0.0;
    /** Timestamp when the reverse started */
    private double reverseStartTime = 0.0;

    public ShooterSubsystem() {
        motor = new TalonFX(Constants.Shooter.MOTOR_ID);
        dutyControl = new DutyCycleOut(0);
        motionMagicVelocity = new MotionMagicVelocityVoltage(0).withSlot(0);
        
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = Constants.Shooter.MOTOR_INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        
        // Slot 0: velocity PID gains (used by Motion Magic Velocity)
        config.Slot0.kV = 0.12;
        config.Slot0.kP = 0.11;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.05;  // Static friction feedforward (volts)
        
        // Motion Magic: acceleration & jerk limits for smooth flywheel spin-up
        // Acceleration: max RPS/sec the profile will command
        config.MotionMagic.MotionMagicAcceleration = 800;  // rps/s — reach 6000 RPM (~100 rps) in ~0.25s
        // Jerk: rps/s² — smooths the start/end of the acceleration ramp
        config.MotionMagic.MotionMagicJerk = 4000;         // rps/s² — snappy but not jarring
        
        motor.getConfigurator().apply(config);
    }

    public boolean checkHealth() {
        isHealthy = motor.isAlive();
        return isHealthy;
    }

    public Boolean isHealthy() {
        return isHealthy;
    }

    public void setManualPower(double power) {
        targetPower = clamp(power, Constants.Shooter.MIN_POWER, Constants.Shooter.MAX_POWER);
        useVelocityControl = false;
        spinUpStartTime = System.currentTimeMillis();
    }
    
    public void setTargetRPM(double rpm) {
        double newRPS = rpm / 60.0;
        double rpmChange = Math.abs(newRPS - targetRPS) * 60.0;
        if (rpmChange > 100.0) {
            spinUpStartTime = System.currentTimeMillis();
        }
        targetRPS = newRPS;
        targetPower = clamp(rpm / (Constants.Shooter.MOTOR_FREE_SPEED_RPS * 60.0), 0.0, 1.0);
        useVelocityControl = true;
    }
    
    public void stop() {
        targetPower = 0.0;
        targetRPS = 0.0;
        isSpunUp = false;
        stallState = StallState.NORMAL;
    }
    
    public boolean isReady() {
        return isSpunUp;
    }
    
    public double getPower() {
        return targetPower;
    }
    
    public double getCurrentPower() {
        return targetPower;
    }
    
    public double getCurrentDistance() {
        return currentDistance;
    }
    
    public double getRPM() {
        return motor.getVelocity().getValueAsDouble() * 60.0;
    }
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            motor.setControl(dutyControl.withOutput(0));
            targetPower = 0.0;
            targetRPS = 0.0;
            isSpunUp = false;
            useVelocityControl = false;
            stallState = StallState.NORMAL;
            DashboardHelper.putNumber(Category.MATCH, "Shooter/TargetRPM", 0.0);
            DashboardHelper.putNumber(Category.MATCH, "Shooter/ActualRPM", 0.0);
            DashboardHelper.putBoolean(Category.MATCH, "Shooter/Ready", false);
            DashboardHelper.putBoolean(Category.DEBUG, "Shooter/Stalled", false);
            return;
        }
        
        double now = Timer.getFPGATimestamp();
        double actualRPM = motor.getVelocity().getValueAsDouble() * 60.0;
        boolean isCommanded = useVelocityControl && targetRPS > 0.1;
        
        // === STALL DETECTION STATE MACHINE ===
        switch (stallState) {
            case NORMAL:
                // Detect stall: motor is commanded to spin but actual RPM is near zero
                if (isCommanded && Math.abs(actualRPM) < Constants.Shooter.STALL_RPM_THRESHOLD) {
                    if (stallStartTime == 0.0) {
                        stallStartTime = now;  // Start the stall timer
                    } else if (now - stallStartTime >= Constants.Shooter.STALL_TIME_SECONDS) {
                        // Confirmed stall — switch to reversing
                        stallState = StallState.REVERSING;
                        reverseStartTime = now;
                        stallStartTime = 0.0;
                        Elastic.sendNotification(new Elastic.Notification()
                            .withLevel(NotificationLevel.WARNING)
                            .withTitle("Shooter JAM!")
                            .withDescription("Stall detected — reversing to clear")
                            .withDisplaySeconds(2.0));
                    }
                } else {
                    stallStartTime = 0.0;  // Reset — motor is spinning fine
                }
                
                // Normal motor control
                if (isCommanded) {
                    motor.setControl(motionMagicVelocity.withVelocity(targetRPS));
                } else {
                    motor.setControl(dutyControl.withOutput(targetPower));
                }
                break;
                
            case REVERSING:
                // Full speed reverse to clear the jam
                motor.setControl(motionMagicVelocity.withVelocity(Constants.Shooter.STALL_REVERSE_RPS));
                
                if (now - reverseStartTime >= Constants.Shooter.STALL_REVERSE_TIME_SECONDS) {
                    // Done reversing — go back to normal
                    stallState = StallState.NORMAL;
                    stallStartTime = 0.0;
                }
                break;
        }
        
        // === READY STATUS ===
        if (stallState == StallState.REVERSING) {
            isSpunUp = false;  // Not ready while clearing a jam
        } else if (isCommanded) {
            double error = Math.abs(actualRPM - targetRPS * 60.0);
            isSpunUp = error < RPM_READY_TOLERANCE;
        } else {
            isSpunUp = false;  // Not commanding velocity = not ready to shoot
        }
        
        // === TELEMETRY ===
        DashboardHelper.putNumber(Category.MATCH, "Shooter/TargetRPM", targetRPS * 60.0);
        DashboardHelper.putNumber(Category.MATCH, "Shooter/ActualRPM", actualRPM);
        DashboardHelper.putNumber(Category.MATCH, "Shooter/Error", Math.abs(actualRPM - targetRPS * 60.0));
        DashboardHelper.putBoolean(Category.MATCH, "Shooter/Ready", isSpunUp);
        DashboardHelper.putBoolean(Category.DEBUG, "Shooter/Stalled", stallState == StallState.REVERSING);
        
        // === TEMPERATURE CHECK ===
        tempCheckCounter++;
        if (tempCheckCounter >= 25) {
            tempCheckCounter = 0;
            double temp = motor.getDeviceTemp().getValueAsDouble();
            if (temp >= TEMP_WARNING_THRESHOLD && !hasWarnedHighTemp) {
                hasWarnedHighTemp = true;
                Elastic.sendNotification(new Elastic.Notification()
                    .withLevel(NotificationLevel.WARNING)
                    .withTitle("Shooter Motor Hot!")
                    .withDescription("Motor at " + (int)temp + "C - consider resting")
                    .withDisplaySeconds(5.0));
            } else if (temp < TEMP_WARNING_THRESHOLD - 10) {
                hasWarnedHighTemp = false;
            }
        }
    }
}
