package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Robot configuration constants. All tunable values in one place.
 */
public final class Constants {

  public static final class Robot {
    public static final double WIDTH_INCHES = 27.0;
    public static final double LENGTH_INCHES = 28.0;
    public static final double WIDTH_METERS = WIDTH_INCHES * 0.0254;
    public static final double LENGTH_METERS = LENGTH_INCHES * 0.0254;
    public static final double HALF_WIDTH_METERS = WIDTH_METERS / 2.0;
    public static final double HALF_LENGTH_METERS = LENGTH_METERS / 2.0;
  }

  /** CAN IDs - Swerve IDs are in TunerConstants */
  public static final class CANIds {
    public static final int TURRET_MOTOR = 13;
    public static final int SHOOTER_TOP_MOTOR = 20;
    public static final int SHOOTER_BOTTOM_MOTOR = 21;
    public static final int TURRET_FEED_MOTOR = 23;
    public static final int INTAKE_PIVOT_MOTOR = 25;
    public static final int INTAKE_ROLLER_MOTOR = 26;
    public static final int INTAKE_CANCODER = 28;
    public static final int CANDLE = 30;
  }

  /** Toggle subsystems for testing */
  public static final class SubsystemEnabled {
    public static final boolean DRIVETRAIN = true;
    public static final boolean TURRET = true;
    public static final boolean SHOOTER = true;
    public static final boolean VISION = true;
    public static final boolean TURRET_FEED = true;
    public static final boolean INTAKE = true;
    public static final boolean LEDS = true;
  }

  public static final class Field {
    public static final double FIELD_LENGTH_METERS = 16.540988;
    public static final double FIELD_WIDTH_METERS = 8.069326;
    
    // Hub position
    public static final double HUB_X = 4.971194;
    public static final double HUB_Y = 4.034631;
    public static final double HUB_HEIGHT_METERS = 1.437087;
    
    // Trench positions
    public static final double TRENCH_ROTATING_X = 4.586034;
    public static final double TRENCH_ROTATING_Y = 0.639458;
    public static final double TRENCH_ROTATING_HEIGHT = 0.681310;
    public static final double TRENCH_FIXED_X = 4.586034;
    public static final double TRENCH_FIXED_Y = 7.421594;
    public static final double TRENCH_FIXED_HEIGHT = 0.676820;
    public static final double TRENCH_HEIGHT_METERS = (TRENCH_ROTATING_HEIGHT + TRENCH_FIXED_HEIGHT) / 2.0;
    
    // Alliance-specific positions
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(HUB_X, HUB_Y);
    public static final Translation2d BLUE_TRENCH_ROTATING = new Translation2d(TRENCH_ROTATING_X, TRENCH_ROTATING_Y);
    public static final Translation2d BLUE_TRENCH_FIXED = new Translation2d(TRENCH_FIXED_X, TRENCH_FIXED_Y);
    public static final Translation2d BLUE_TRENCH_TARGET = BLUE_TRENCH_ROTATING;
    
    public static final Translation2d RED_HUB_CENTER = new Translation2d(FIELD_LENGTH_METERS - HUB_X, FIELD_WIDTH_METERS - HUB_Y);
    public static final Translation2d RED_TRENCH_ROTATING = new Translation2d(FIELD_LENGTH_METERS - TRENCH_ROTATING_X, FIELD_WIDTH_METERS - TRENCH_ROTATING_Y);
    public static final Translation2d RED_TRENCH_FIXED = new Translation2d(FIELD_LENGTH_METERS - TRENCH_FIXED_X, FIELD_WIDTH_METERS - TRENCH_FIXED_Y);
    public static final Translation2d RED_TRENCH_TARGET = RED_TRENCH_ROTATING;

    public static final boolean OVERRIDE_FMS_CHECK = false;
    public static final double AUTO_SHUTTLE_BOUNDARY_X = 4.0;
    public static final boolean AUTO_SHUTTLE_ENABLED = true;
  }

  // Shooter constants
  public static final class Shooter {
    public static final int TOP_MOTOR_ID = CANIds.SHOOTER_TOP_MOTOR;
    public static final int BOTTOM_MOTOR_ID = CANIds.SHOOTER_BOTTOM_MOTOR;
    public static final boolean TOP_MOTOR_INVERTED = false;
    public static final boolean BOTTOM_MOTOR_INVERTED = true;
    public static final double WHEEL_RADIUS_METERS = 0.0508;
    public static final double MAX_POWER = 1.0;
    public static final double MIN_POWER = 0.0;
    public static final double SPIN_UP_TIME_SECONDS = 0.5;
    public static final double TOP_MOTOR_POWER_OFFSET = 0.0;
    public static final double BOTTOM_MOTOR_POWER_OFFSET = 0.0;
    
    /** Idle RPM for both motors when not actively spooling. 
     *  Keeps flywheels warm so spin-up is faster. Tunable via SmartDashboard "Shooter/IdleRPM". 
     *  Set to 0 to fully stop when idle. */
    public static final double DEFAULT_IDLE_RPM = 500.0;
    
    /** Free speed of the motor in rotations per second (Kraken X60 ~100 rps) */
    public static final double MOTOR_FREE_SPEED_RPS = 100.0;
    
    // Hub calibration: distance (m) -> {TopRPM, BottomRPM}
    // Values are in motor RPM (Kraken X60 free speed ~6000 RPM)
    // Reduced ~25% from original values — was overshooting at all distances
    public static final TreeMap<Double, double[]> SHOOTING_CALIBRATION = new TreeMap<>() {{
      put(2.00, new double[]{500, 2500}); put(3.5, new double[]{2000,2000});
      put(5.75, new double[]{1500, 4000});
    }};
    
    // Trench calibration for shuttle shots: distance (m) -> {TopRPM, BottomRPM}
    public static final TreeMap<Double, double[]> TRENCH_CALIBRATION = new TreeMap<>() {{
      put(2.0, new double[]{1500, 3000}); put(3.0, new double[]{1800, 3300});
      put(4.0, new double[]{2100, 3600}); put(5.0, new double[]{2400, 3900});
      put(6.0, new double[]{2700, 4200}); put(7.0, new double[]{3000, 4500});
      put(8.0, new double[]{3300, 4800});
    }};
  }

  public static final class TurretFeed {
    public static final int MOTOR_ID = CANIds.TURRET_FEED_MOTOR;
    public static final boolean MOTOR_INVERTED = false;
    public static final double IDLE_SPEED = 0.15;
    public static final double SHOOT_SPEED = -1.0; // Negative = reverse direction to feed balls into shooter
    public static final double STOP_SPEED = 0.0;
  }

  public static final class Intake {
    public static final int PIVOT_MOTOR_ID = CANIds.INTAKE_PIVOT_MOTOR;
    public static final int ROLLER_MOTOR_ID = CANIds.INTAKE_ROLLER_MOTOR;
    public static final int CANCODER_ID = CANIds.INTAKE_CANCODER;
    public static final boolean PIVOT_MOTOR_INVERTED = true;
    public static final boolean ROLLER_MOTOR_INVERTED = true;
    public static final double CANCODER_OFFSET_DEG = -100.107;
    public static final double PIVOT_GEAR_RATIO = 45.0;
    
    // Pivot PID (WPILib PID -- works in degrees, outputs duty cycle)
    // P: gets it moving toward target
    // D: currently 0 -- increase if overshoot/bounce is a problem
    public static final double PIVOT_PID_P = 0.008;
    public static final double PIVOT_PID_I = 0.00;
    public static final double PIVOT_PID_D = 0.0;
    
    // Pivot limits
    public static final double MIN_PIVOT_ANGLE_DEG = 0.0;
    public static final double MAX_PIVOT_ANGLE_DEG = 90.0;
    public static final double RETRACTED_ANGLE_DEG = 0.0;
    public static final double DEPLOYED_ANGLE_DEG = 120;
    public static final double IDLE_ANGLE_DEG = 35;
    public static final double PIVOT_MAX_OUTPUT = 0.3;
    // Wide tolerance to stop before chain slop causes bouncing
    public static final double PIVOT_TOLERANCE_DEG = 15.0;
    
    // Roller
    public static final double INTAKE_SPEED = 0.8;
    public static final double OUTTAKE_SPEED = -0.6;
    public static final double STOP_SPEED = 0.0;
  }

  /** PhotonVision camera configuration */
  public static final class Vision {
    /** Camera name as configured in PhotonVision UI (http://photonvision.local:5800) */
    public static final String CAMERA_NAME = "Arducam_OV9782_USB_Camera";
    
    // Camera mounting position relative to robot center (meters)
    // Camera is at the front of the robot, about 3 inches behind the front edge
    /** Forward/backward from robot center (+ = forward) */
    public static final double CAMERA_X_OFFSET = (Robot.LENGTH_INCHES / 2.0 - 3.0) * 0.0254; // ~11 inches forward = ~0.28m
    /** Left/right from robot center (+ = left) */
    public static final double CAMERA_Y_OFFSET = 0.0; // Centered left-right
    /** Height from ground to camera lens */
    public static final double CAMERA_Z_OFFSET = 0.5;
    /** Tilt angle in degrees (+ = tilted up) */
    public static final double CAMERA_PITCH_DEGREES = 15.0; // Slight upward pitch
    /** Yaw rotation in degrees (0 = facing forward, 90 = facing left, 180 = facing backward) */
    public static final double CAMERA_YAW_DEGREES = 0.0; // Camera faces FORWARD on robot
    
    // Vision measurement trust (standard deviations for pose estimator)
    // Lower = trust vision more, Higher = trust odometry more
    // Balanced trust — vision corrects drift but odometry keeps things smooth.
    // Was 0.1/0.1/0.1 which caused jitter from noisy vision updates.
    public static final double VISION_STD_DEV_X = 0.7;
    public static final double VISION_STD_DEV_Y = 0.7;
    public static final double VISION_STD_DEV_THETA = 10;
    
    /** Maximum pose ambiguity to accept a vision measurement (0-1, lower = stricter) */
    public static final double MAX_AMBIGUITY = 0.1;
  }

  public static final class Driver {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double STICK_DEADBAND = 0.1;
    public static final double MAX_DRIVE_SPEED_MPS = 4.5;
    public static final double MAX_ANGULAR_SPEED_RAD = Math.PI * 2;
    public static final double SLOW_MODE_MULTIPLIER = 0.3;
  }
  
  public static final class Turret {
    public static final int MOTOR_ID = CANIds.TURRET_MOTOR;
    public static final double GEAR_RATIO = 10.00537109375; // Calibrate with CalibrateTurretGearRatioCommand
    public static final boolean MOTOR_INVERTED = true;
    
    // Bounded 0-360 position system. STARTUP_ANGLE is internal position at power-on.
    // HOME_ANGLE_DEG is the EXTERNAL angle the turret physically faces at power-on.
    //   It tells the code "at power-on, the turret is pointing at this external angle."
    //   External 0 = robot forward. External 90 = robot left. External -90 = robot right.
    //   Since the turret physically starts facing left, HOME_ANGLE_DEG = 90.
    //
    // MOTOR_INVERTED = true because the motor's physical CW = positive encoder,
    //   but the math convention needs positive internal angle change to go CCW.
    //   Phoenix 6 inversion flips BOTH encoder and output together, keeping math consistent.
    //
    // MIN/MAX are in INTERNAL coordinates (0-360).
    //   Internal 90 = forward, Internal 180 = home/left (startup).
    //   Usable range WRAPS through 0/360: from MIN(75) → 0/360 → through 90,180 → to MAX(224).
    //   Dead zone: MAX(224) → through 270,315 → MIN(75) (cannot traverse).
    public static final double STARTUP_ANGLE_DEG = 180.0; // Internal angle at power-on (arbitrary reference)
    public static final double HOME_ANGLE_DEG = 90.0;     // External angle at power-on (90 = facing left)
    public static final double MIN_ANGLE_DEG = 75.0;      // CW internal limit
    public static final double MAX_ANGLE_DEG = 224.0;     // CCW internal limit

    public static final double LIMIT_SAFETY_MARGIN_DEG = 10.0;
    
    // PID
    public static final double PID_P = 0.2;
    public static final double PID_I = 0.0;
    public static final double PID_D = 0.01;
    
    // Motion
    public static final double GLOBAL_ANGLE_OFFSET_DEG = 0.0;
    public static final double MAX_ANGULAR_SPEED_DEG_PER_SEC = 90.0;
    public static final double MAX_OUTPUT = 0.3;
    
    // Position on robot (6.25" from back, 6.25" from left edge)
    public static final double TURRET_FROM_BACK_INCHES = 6.25;
    public static final double TURRET_FROM_LEFT_INCHES = 6.25;
    public static final double TURRET_X_OFFSET = (TURRET_FROM_BACK_INCHES - (Robot.LENGTH_INCHES / 2.0)) * 0.0254;
    public static final double TURRET_Y_OFFSET = ((Robot.WIDTH_INCHES / 2.0) - TURRET_FROM_LEFT_INCHES) * 0.0254;
    
    public static final double ON_TARGET_TOLERANCE_DEG = 20.0;
    
    // Distance-based turret angle offset calibration: distance (m) -> angle offset (deg)
    // At each distance, this offset is added to the calculated turret angle to correct aim.
    // Use the "Cal: Turret Rotation" command to record offsets, then paste the output here.
    public static final TreeMap<Double, Double> ROTATION_CALIBRATION = new TreeMap<>() {{
      put(2.0, 0.0); put(3.5, 0.0);
    }};
    
    // Per-tag calibration offsets (initialize all to 0)
    public static final Map<Integer, Double> TAG_ANGLE_OFFSETS = new HashMap<>() {{ for (int i = 1; i <= 32; i++) put(i, 0.0); }};
    public static final Map<Integer, Double> TAG_DISTANCE_OFFSETS = new HashMap<>() {{ for (int i = 1; i <= 32; i++) put(i, 0.0); }};
  }
  
  public static final class Tags {
    public static final int[] BLUE_HUB = {18, 19, 20, 21, 24, 25, 26, 27};
    public static final int[] BLUE_TRENCH = {17, 22, 23, 28};
    public static final int[] RED_HUB = {2, 3, 4, 5, 8, 9, 10, 11};
    public static final int[] RED_TRENCH = {1, 6, 7, 12};
  }
  
  public static final class LEDs {
    public static final int CANDLE_ID = CANIds.CANDLE;
    public static final int LED_COUNT = 65;
    public static final int ONBOARD_LED_COUNT = 8;   // CANdle's built-in LEDs (indices 0-7)
    public static final int STRIP_START = 8;          // External strip starts at index 8
    public static final int STRIP_COUNT = LED_COUNT - ONBOARD_LED_COUNT;  // 60 strip LEDs
    public static final double BOOT_ANIMATION_DURATION = 2.0;
    public static final double CHASE_SPEED = 0.15;
    public static final double BRIGHTNESS = 0.8;
    
    // Colors are RGB format {Red, Green, Blue}
    // Strip type is GRB (set in LEDSubsystem CANdle config)
    
    // Team colors
    public static final int[] TEAM_SAFETY_ORANGE = {255, 100, 0};
    public static final int[] TEAM_BLUE = {0, 100, 255};
    
    // Alliance
    public static final int[] BLUE_ALLIANCE = {0, 0, 255};
    public static final int[] RED_ALLIANCE = {255, 0, 0};
    
    // Basic colors
    public static final int[] GREEN = {0, 255, 0};
    public static final int[] ORANGE = {255, 165, 0};
    public static final int[] PURPLE = {128, 0, 128};
    public static final int[] YELLOW = {255, 255, 0};
    public static final int[] WHITE = {255, 255, 255};
    public static final int[] OFF = {0, 0, 0};
    public static final int[] CYAN = {0, 255, 255};
    public static final int[] MAGENTA = {255, 0, 255};
    public static final int[] PINK = {255, 105, 180};
    
    // Status colors
    public static final int[] NO_AUTO_COLOR = {255, 165, 0};
    public static final int[] FMS_DISCONNECTED_COLOR = {255, 255, 0};
    
    // Action colors
    public static final int[] SHOOTING_COLOR = {0, 255, 0};
    public static final int[] AIMING_COLOR = {255, 255, 0};
    public static final int[] SPOOLING_COLOR = {255, 100, 0};
    public static final int[] INTAKING_COLOR = {255, 100, 180};
    public static final int[] CLIMBING_COLOR = {255, 0, 255};
    public static final int[] ESTOP_COLOR = {255, 0, 0};
    public static final int[] BROWNOUT_COLOR = {64, 28, 0};  // Saddle brown
    
    // Warning colors
    public static final int[] SHIFT_WARNING_5SEC = {255, 255, 255};
    public static final int[] SHIFT_WARNING_3SEC = {255, 255, 0};
    public static final int[] ENDGAME_WARNING_10SEC = {255, 100, 0};
    public static final int[] ENDGAME_WARNING_5SEC = {255, 0, 255};
    public static final int[] ENDGAME_WARNING_3SEC = {255, 0, 255};
    
    // Victory
    public static final int[] VICTORY_COLOR_1 = {255, 100, 0};
    public static final int[] VICTORY_COLOR_2 = {0, 100, 255};
    public static final double VICTORY_FLASH_SPEED = 0.9; // Fast strobe (10Hz)
  }
}
