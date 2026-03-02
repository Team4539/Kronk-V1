package frc.robot;

import java.util.ArrayList;
import java.util.List;

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
    public static final int SHOOTER_MOTOR = 20;
    public static final int TRIGGER_MOTOR = 23;
    public static final int INTAKE_PIVOT_MOTOR = 25;
    public static final int INTAKE_ROLLER_MOTOR = 26;
    public static final int INTAKE_CANCODER = 28;
    public static final int CANDLE = 30;
  }

  /** Toggle subsystems for testing */
  public static final class SubsystemEnabled {
    public static final boolean DRIVETRAIN = true;
    public static final boolean SHOOTER = true;
    public static final boolean VISION = true;
    public static final boolean TRIGGER = true;
    public static final boolean INTAKE = true;
    public static final boolean LEDS = true;
  }

  public static final class Field {
    public static final double FIELD_LENGTH_METERS = 16.540988;
    public static final double FIELD_WIDTH_METERS = 8.069326;
    
    // Hub position
    public static final double HUB_X = 4.571194;
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

  // Shooter constants (single motor fixed shooter)
  public static final class Shooter {
    public static final int MOTOR_ID = CANIds.SHOOTER_MOTOR;
    public static final boolean MOTOR_INVERTED = false;
    public static final double WHEEL_RADIUS_METERS = 0.0508;
    public static final double MAX_POWER = 1.0;
    public static final double MIN_POWER = 0.0;
    public static final double SPIN_UP_TIME_SECONDS = 0.5;
    public static final double MOTOR_POWER_OFFSET = 0.0;
    
    // --- Ball properties (2026 game piece) ---
    public static final double BALL_DIAMETER_METERS = 0.1500;
    public static final double BALL_RADIUS_METERS = BALL_DIAMETER_METERS / 2.0;
    public static final double BALL_MASS_KG = 0.215;
    public static final double BALL_COMPRESSION_METERS = 0.0135;
    public static final double BALL_COMPRESSED_RADIUS_METERS = BALL_RADIUS_METERS - (BALL_COMPRESSION_METERS / 2.0);
    public static final double SHOOTER_EXIT_HEIGHT_METERS = 0.50;
    
    /** Idle RPM when not actively spooling. Keeps flywheel warm for faster spin-up. */
    public static final double DEFAULT_IDLE_RPM = 500.0;
    
    /** Free speed of the motor in rotations per second (Kraken X60 ~100 rps) */
    public static final double MOTOR_FREE_SPEED_RPS = 100.0;
    
    // --- Shooting Calibration ---
    //
    // Distance-based calibration table for the fixed shooter.
    // Each entry: {relX, relY, bearingDeg, shooterRPM}
    //   - relX:       Robot X relative to target (meters, robotX - targetX)
    //   - relY:       Robot Y relative to target (meters, robotY - targetY)
    //   - bearingDeg: Robot-relative angle to target (degrees, -180 to +180)
    //   - shooterRPM: Shooter motor RPM at this pose
    //
    // At runtime, ShootingCalculator computes the robot's current (relX, relY, bearing)
    // and uses inverse-distance-weighted interpolation to produce the RPM.
    //
    public static final double CALIBRATION_BEARING_WEIGHT = 0.05;
    public static final List<double[]> SHOOTING_CALIBRATION = new ArrayList<>() {{
      add(new double[]{2.81, 0.76, 19, 2500}); add(new double[]{0.79, 1.95, 62, 2500});
      add(new double[]{3.41, 0.46, 4, 2500}); add(new double[]{2.29, 1.76, 81, 2500});
      add(new double[]{4.61, -0.27, 7, 3050}); add(new double[]{5.32, 1.41, 38, 3050});
      add(new double[]{3.22, -0.18, 13, 2500}); add(new double[]{1.99, 0.61, 30, 2500});
      add(new double[]{2.48, 1.32, 35, 2600}); add(new double[]{3.76, 1.13, 4, 2500});
    }};
  }

  public static final class Trigger {
    public static final int MOTOR_ID = CANIds.TRIGGER_MOTOR;
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
    // Pivot PID (WPILib PID, works in degrees, outputs duty cycle)
    public static final double PIVOT_PID_P = 0.02;
    public static final double PIVOT_PID_I = 0.000;
    public static final double PIVOT_PID_D = 0.00;
    
    // Pivot limits
    public static final double MIN_PIVOT_ANGLE_DEG = 0.0;
    public static final double MAX_PIVOT_ANGLE_DEG = 130.0;
    public static final double RETRACTED_ANGLE_DEG = 0.0;
    public static final double DEPLOYED_ANGLE_DEG = 130;
    public static final double IDLE_ANGLE_DEG = 20;
    public static final double PIVOT_MAX_OUTPUT = 0.3;
    // Wider tolerance to prevent oscillation from mechanical slop
    public static final double PIVOT_TOLERANCE_DEG = 25.0;
    
    // Roller
    public static final double INTAKE_SPEED = 0.6;
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
    public static final double CAMERA_X_OFFSET = 0.2032;
    /** Left/right from robot center (+ = left) */
    public static final double CAMERA_Y_OFFSET = -0.1524;
    /** Height from ground to camera lens */
    public static final double CAMERA_Z_OFFSET = 0.23495 ;
    /** Tilt angle in degrees (+ = tilted up) */
    public static final double CAMERA_PITCH_DEGREES = 38;
    /** Yaw rotation in degrees (0 = facing forward, 90 = facing left, 180 = facing backward) */
    public static final double CAMERA_YAW_DEGREES = 0.0; // Camera faces FORWARD on robot
    
    // Vision measurement trust (standard deviations for pose estimator)
    // Lower = trust vision more, Higher = trust odometry more
    public static final double VISION_STD_DEV_X = 0.7;
    public static final double VISION_STD_DEV_Y = 0.7;
    public static final double VISION_STD_DEV_THETA = 10;
    
    /** Maximum pose ambiguity to accept a vision measurement (0-1, lower = stricter) */
    public static final double MAX_AMBIGUITY = 0.1;
  }

  public static final class Driver {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double STICK_DEADBAND = 0.1;
    public static final double MAX_DRIVE_SPEED_MPS = 4.5;
    public static final double MAX_ANGULAR_SPEED_RAD = Math.PI * 2;
    public static final double SLOW_MODE_MULTIPLIER = 0.3;
  }
  
  public static final class Tags {
    public static final int[] BLUE_HUB = {18, 19, 20, 21, 24, 25, 26, 27};
    public static final int[] BLUE_TRENCH = {17, 22, 23, 28};
    public static final int[] RED_HUB = {2, 3, 4, 5, 8, 9, 10, 11};
    public static final int[] RED_TRENCH = {1, 6, 7, 12};
  }
  
  public static final class LEDs {
    public static final int CANDLE_ID = CANIds.CANDLE;
    
    // === PHYSICAL LAYOUT ===
    // The CANdle's data line is Y-spliced to TWO physical strips:
    //   - Strip A (top/back of robot): 38 LEDs
    //   - Strip B (belly pan):         ~58 LEDs (same data line, continues after Strip A's length)
    //
    // Because they share the same data signal:
    //   Indices 0-7:   CANdle onboard LEDs (visible on belly pan)
    //   Indices 8-45:  "SHARED ZONE" — these 38 LEDs light up on BOTH strips simultaneously
    //                  (Strip A top/back shows all 38, Strip B belly shows first 38)
    //   Indices 46-65: "BELLY ONLY ZONE" — these ~20 LEDs only exist on Strip B (belly pan)
    //                  Strip A has no LEDs at these addresses.
    //
    // For animations: the shared zone (8-45) is the main show — it's visible from
    // both top and bottom. The belly-only zone (46-65) is bonus visibility underneath.
    
    public static final int ONBOARD_LED_COUNT = 8;    // CANdle built-in LEDs (indices 0-7), belly pan
    public static final int SHARED_STRIP_COUNT = 38;  // Mirrored on BOTH top strip and belly strip
    public static final int BELLY_ONLY_COUNT = 20;    // Only on belly strip (after shared zone)
    
    // Computed layout indices
    public static final int STRIP_START = ONBOARD_LED_COUNT;                          // 8
    public static final int SHARED_END = STRIP_START + SHARED_STRIP_COUNT - 1;        // 45
    public static final int BELLY_START = SHARED_END + 1;                             // 46
    public static final int BELLY_END = BELLY_START + BELLY_ONLY_COUNT - 1;           // 65
    
    public static final int LED_COUNT = ONBOARD_LED_COUNT + SHARED_STRIP_COUNT + BELLY_ONLY_COUNT; // 66
    public static final int STRIP_COUNT = SHARED_STRIP_COUNT + BELLY_ONLY_COUNT;      // 58 total strip LEDs
    
    public static final double BOOT_ANIMATION_DURATION = 2.5;
    public static final double CHASE_SPEED = 0.15;
    public static final double BRIGHTNESS = 0.8;
    
    // Colors are RGB format {Red, Green, Blue}
    // Strip type is GRB (set in LEDSubsystem CANdle config)
    
    // ── Team Identity ──
    public static final int[] TEAM_SAFETY_ORANGE = {255, 100, 0};
    public static final int[] TEAM_BLUE = {0, 100, 255};
    public static final int[] TEAM_ORANGE_WARM = {255, 60, 0};   // Deeper ember orange
    public static final int[] TEAM_BLUE_DEEP = {0, 50, 200};     // Deeper blue accent
    public static final int[] TEAM_GOLD = {255, 180, 30};        // Gold accent for highlights
    
    // ── Alliance ──
    public static final int[] BLUE_ALLIANCE = {0, 0, 255};
    public static final int[] RED_ALLIANCE = {255, 0, 0};
    public static final int[] BLUE_ALLIANCE_DIM = {0, 0, 60};
    public static final int[] RED_ALLIANCE_DIM = {60, 0, 0};
    
    // ── Core Palette ──
    public static final int[] GREEN = {0, 255, 0};
    public static final int[] ORANGE = {255, 165, 0};
    public static final int[] PURPLE = {128, 0, 128};
    public static final int[] YELLOW = {255, 255, 0};
    public static final int[] WHITE = {255, 255, 255};
    public static final int[] WARM_WHITE = {255, 220, 180};      // Soft warm glow
    public static final int[] COOL_WHITE = {200, 220, 255};      // Ice-cool accent
    public static final int[] OFF = {0, 0, 0};
    public static final int[] CYAN = {0, 255, 255};
    public static final int[] MAGENTA = {255, 0, 255};
    public static final int[] PINK = {255, 105, 180};
    public static final int[] HOT_PINK = {255, 20, 100};         // Vivid intake accent
    public static final int[] ELECTRIC_BLUE = {30, 144, 255};    // Bright electric accent
    
    // ── Status ──
    public static final int[] NO_AUTO_COLOR = {255, 120, 0};
    public static final int[] FMS_DISCONNECTED_COLOR = {255, 200, 0};
    
    // ── Action ──
    public static final int[] SHOOTING_COLOR = {0, 255, 60};     // Vivid green with cyan tint
    public static final int[] SHOOTING_HIGHLIGHT = {180, 255, 200}; // Bright mint for leading edges
    public static final int[] AIMING_COLOR = {255, 200, 0};      // Amber-gold targeting
    public static final int[] AIMING_HIGHLIGHT = {255, 255, 180}; // Bright yellow-white lock flash
    public static final int[] SPOOLING_COLOR = {255, 80, 0};     // Deep charging orange
    public static final int[] SPOOLING_HIGHLIGHT = {255, 200, 100}; // Hot white-orange leading edge
    public static final int[] INTAKING_COLOR = {255, 40, 120};   // Vivid magenta-pink
    public static final int[] INTAKING_HIGHLIGHT = {255, 180, 220}; // Soft pink-white tip
    public static final int[] CLIMBING_COLOR = {180, 0, 255};    // Rich violet
    public static final int[] CLIMBING_HIGHLIGHT = {220, 140, 255}; // Lavender tip
    public static final int[] ESTOP_COLOR = {255, 0, 0};
    public static final int[] ESTOP_CORE = {255, 40, 0};         // Orange-red hot core
    public static final int[] BROWNOUT_COLOR = {80, 35, 0};      // Warmer brown
    public static final int[] BROWNOUT_SPARK = {255, 160, 40};   // Bright amber spark
    
    // ── Warnings ──
    public static final int[] SHIFT_WARNING_5SEC = {255, 255, 255};
    public static final int[] SHIFT_WARNING_3SEC = {255, 255, 0};
    public static final int[] ENDGAME_WARNING_10SEC = {255, 100, 0};
    public static final int[] ENDGAME_WARNING_5SEC = {255, 0, 255};
    public static final int[] ENDGAME_WARNING_3SEC = {255, 0, 255};
    
    // ── Victory ──
    public static final int[] VICTORY_GOLD = {255, 200, 50};     // Bright gold burst
    public static final int[] VICTORY_COLOR_1 = {255, 100, 0};
    public static final int[] VICTORY_COLOR_2 = {0, 100, 255};
    public static final double VICTORY_FLASH_SPEED = 0.9;
  }
}
