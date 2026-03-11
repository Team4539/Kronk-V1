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

    // Shuttle targets — alliance zone side positions (shoot to the wall side of your zone)
    // Blue: left side of field (low X), near the bottom side wall (low Y)
    // Red: right side of field (high X), near the top side wall (high Y), mirrored
    public static final double SHUTTLE_X = 2.0;                     // ~2m into blue alliance zone
    public static final double SHUTTLE_Y = 4.034631;                     // Near side wall
    public static final Translation2d BLUE_SHUTTLE_TARGET = new Translation2d(SHUTTLE_X, SHUTTLE_Y);
    public static final Translation2d RED_SHUTTLE_TARGET = new Translation2d(FIELD_LENGTH_METERS - SHUTTLE_X, FIELD_WIDTH_METERS - SHUTTLE_Y);

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
    /** Fallback RPM used when vision/pose is unavailable and driver manually ranges shots. */
    public static final double FALLBACK_RPM = 6000.0;
    
    /** Free speed of the motor in rotations per second (Kraken X60 ~100 rps) */
    public static final double MOTOR_FREE_SPEED_RPS = 100.0;
    
    // --- Stall detection & auto-reverse ---
    // If the shooter is commanded to spin but actual RPM stays near 0 for
    // STALL_TIME_SECONDS, it reverses at full speed for STALL_REVERSE_TIME_SECONDS
    // to clear a jam, then resumes the commanded RPM.
    public static final double STALL_RPM_THRESHOLD = 50.0;         // Below this = not spinning
    public static final double STALL_TIME_SECONDS = 1.0;           // How long at 0 RPM before stall confirmed
    public static final double STALL_REVERSE_TIME_SECONDS = 0.5;   // How long to reverse to clear jam
    public static final double STALL_REVERSE_RPS = -100.0;         // Full speed reverse (rotations/sec)
    
    // --- Shooting Calibration ---
    //
    // Simple distance-based calibration table for the fixed shooter.
    // Each entry: {distanceMeters, shooterRPM}
    //   - distanceMeters: Straight-line distance from robot to target (meters)
    //   - shooterRPM:     Shooter motor RPM at this distance
    //
    // At runtime, ShootingCalculator computes the robot's distance to the target
    // and linearly interpolates between the two nearest calibration points.
    // The table should be sorted by distance (smallest first).
    //
    public static final List<double[]> SHOOTING_CALIBRATION = new ArrayList<>() {{
      add(new double[]{0.60, 2250}); add(new double[]{3.01, 3175}); add(new double[]{5.31, 4175});
      
    }};
  }

  public static final class Trigger {
    public static final int MOTOR_ID = CANIds.TRIGGER_MOTOR;
    public static final boolean MOTOR_INVERTED = false;
    /** Idle RPM — slow feed to stage balls near the shooter */
    public static final double IDLE_SPEED_RPM = 2250;
    /** Shoot RPM — full speed reverse to feed balls into shooter (negative = reverse) */
    public static final double SHOOT_SPEED_RPM = -3000.0;
  }

  public static final class Intake {
    public static final int PIVOT_MOTOR_ID = CANIds.INTAKE_PIVOT_MOTOR;
    public static final int ROLLER_MOTOR_ID = CANIds.INTAKE_ROLLER_MOTOR;
    public static final int CANCODER_ID = CANIds.INTAKE_CANCODER;
    public static final boolean PIVOT_MOTOR_INVERTED = true;
    public static final boolean ROLLER_MOTOR_INVERTED = true;
    public static final double CANCODER_OFFSET_DEG = 0;
    public static final double PIVOT_GEAR_RATIO = 81.0;
  
    // Pivot limits
    public static final double MIN_PIVOT_ANGLE_DEG = 95.4;  // cancoder got moved old value 59.7
    public static final double MAX_PIVOT_ANGLE_DEG = 202.7;
    public static final double RETRACTED_ANGLE_DEG = 63.7;
    public static final double DEPLOYED_ANGLE_DEG = 208.7;  // Must be <= MAX_PIVOT_ANGLE_DEG
    public static final double IDLE_ANGLE_DEG =  60.0;
    public static final double HALF_SHOOT_ANGLE_DEG = 90.0;
    // Tolerance for "at target" check (degrees)
    public static final double PIVOT_TOLERANCE_DEG = 2.0;
    

    // Jiggle: oscillate pivot while shooting to force balls into the shooter.
    // Sweeps from near-retracted to near-deployed with rollers spinning inward.
    public static final double JIGGLE_CYCLE_SECONDS = 0.35;  // Full cycle time (aggressive pump action)
    public static final double JIGGLE_MIN_ANGLE_DEG = 65.0;  // Near retracted — where balls feed
    public static final double JIGGLE_MAX_ANGLE_DEG = 150.0; // Near deployed — pushes balls back down
    public static final double JIGGLE_ROLLER_RPM = 6000.0;   // Rollers spin inward to force-feed balls

    // Roller (RPM-based, controlled via Motion Magic Velocity)
    public static final double INTAKE_SPEED_RPM = 6000.0;
    public static final double OUTTAKE_SPEED_RPM = -3000.0;
    public static final double STOP_SPEED = 0.0;

    // Stall detection & auto-reverse
    // If supply current exceeds threshold AND roller velocity drops below threshold
    // for STALL_TIME_SECONDS, the roller reverses at full speed for REVERSE_TIME_SECONDS.
    public static final double STALL_CURRENT_THRESHOLD_AMPS = 35.0;  // Supply current indicating stall
    public static final double STALL_VELOCITY_THRESHOLD_RPS = 10.0;  // Below this = not spinning (rot/s)
    public static final double STALL_TIME_SECONDS = 0.15;            // How long stall conditions must hold
    public static final double REVERSE_TIME_SECONDS = 0.5;           // How long to reverse after stall
    public static final double REVERSE_SPEED_RPM = -6000.0;          // Full reverse speed
  }

  /** PhotonVision camera configuration */
  public static final class Vision {
    // === FRONT CAMERA ===
    // Mounted high up, dead center on the robot, pointing forward and slightly up.
    /** Camera name as configured in PhotonVision UI (http://photonvision.local:5800) */
    public static final String FRONT_CAMERA_NAME = "Arducam_OV9782_USB_Camera";
    
    // Front camera mounting position relative to robot center (meters)
    /** Forward/backward from robot center (+ = forward) — centered */
    public static final double FRONT_CAMERA_X = 0.0;
    /** Left/right from robot center (+ = left) — centered */
    public static final double FRONT_CAMERA_Y = 0.0;
    /** Height from ground to camera lens (~20 inches) */
    public static final double FRONT_CAMERA_Z = 0.508;
    /** Pitch in degrees (+ = tilted up from horizontal) — slightly up */
    public static final double FRONT_CAMERA_PITCH_DEG = 5.0;
    /** Yaw in degrees (0 = facing forward) */
    public static final double FRONT_CAMERA_YAW_DEG = 180;
    
    // Vision measurement trust (standard deviations for pose estimator)
    // Lower = trust vision more, Higher = trust odometry more
    public static final double VISION_STD_DEV_X = 0.7;
    public static final double VISION_STD_DEV_Y = 0.7;
    public static final double VISION_STD_DEV_THETA = 0.5;  // Trust camera rotation for auto-aim
    
    /** Maximum pose ambiguity to accept a vision measurement (0-1, lower = stricter) */
    public static final double MAX_AMBIGUITY = 0.1;
  }

  public static final class Driver {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double STICK_DEADBAND = 0.1;
    public static final double MAX_DRIVE_SPEED_MPS = 4.5;
    public static final double MAX_ANGULAR_SPEED_RAD = Math.PI * 2;
    public static final double SLOW_MODE_MULTIPLIER = 0.3;
    
    // Auto-aim heading PD controller (rotation rate = KP * error - KD * errorRate)
    // Controls how aggressively the robot rotates to face the target.
    // KP drives toward the target; KD damps oscillation as the error shrinks.
    // Inside AIM_SLOW_ZONE_DEG, KP scales down so the robot decelerates smoothly.
    // Output is clamped to MAX_ANGULAR_SPEED_RAD so large errors don't command unsafe rates.
    public static final double AIM_HEADING_KP = 15;    // Proportional (rad/s per radian of error)
    public static final double AIM_HEADING_KD = 0.03;   // Derivative (rad/s per rad/s of error change)
    /** Below this error (degrees), output zero — prevents micro-oscillation jitter near target */
    public static final double AIM_DEADBAND_DEG = 0.0;
    /** When error is within this zone (degrees), KP ramps down for a smooth approach */
    public static final double AIM_SLOW_ZONE_DEG = 0.0;
    /** Angle tolerance in degrees — "aimed" when error is within this */
    public static final double AIM_TOLERANCE_DEG = .25;
    /** 
     * Sign multiplier for auto-aim rotation direction.
     * +1.0 = normal (positive angleToTarget → CCW rotation)
     * -1.0 = inverted (use if Pigeon mount compensation isn't working)
     * 
     * If the robot rotates AWAY from the target, flip this to -1.0.
     */
    public static final double AIM_DIRECTION = 1.0;
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
    // The CANdle's data line drives a single back/top strip only.
    // The belly pan strip is disconnected (broken).
    //
    //   Indices 0-7:   CANdle onboard LEDs
    //   Indices 8-45:  Back/top strip: 38 LEDs
    //
    // No belly-only zone — those LEDs are gone.
    
    public static final int ONBOARD_LED_COUNT = 8;    // CANdle built-in LEDs (indices 0-7)
    public static final int SHARED_STRIP_COUNT = 38;  // Back/top strip only
    public static final int BELLY_ONLY_COUNT = 0;     // Belly strip is disconnected (broken)
    
    // Computed layout indices
    public static final int STRIP_START = ONBOARD_LED_COUNT;                          // 8
    public static final int SHARED_END = STRIP_START + SHARED_STRIP_COUNT - 1;        // 45
    public static final int BELLY_START = SHARED_END + 1;                             // 46
    public static final int BELLY_END = BELLY_START + BELLY_ONLY_COUNT - 1;           // 45 (empty range)
    
    public static final int LED_COUNT = ONBOARD_LED_COUNT + SHARED_STRIP_COUNT + BELLY_ONLY_COUNT; // 46
    public static final int STRIP_COUNT = SHARED_STRIP_COUNT + BELLY_ONLY_COUNT;      // 38 total strip LEDs
    
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
    // STRATEGY: Each state uses a completely different hue family so they
    // are instantly distinguishable even through smoky/translucent panels.
    //   SHOOT NOW  = solid GREEN + WHITE strobe    (GO! GO! GO!)
    //   FIRING     = RED + ORANGE explosive fire   (muzzle blast)
    //   AIMING     = PURPLE/VIOLET radar sweep     (targeting)
    //   SPOOLING   = ORANGE power fill             (charging up)
    //   INTAKING   = YELLOW-GREEN inward flow      (sucking in)
    //   CLIMBING   = DEEP BLUE rising flames       (upward effort)
    public static final int[] SHOOTING_COLOR = {0, 255, 0};      // Pure vivid green
    public static final int[] SHOOTING_HIGHLIGHT = {255, 255, 255}; // Bright white for strobing
    public static final int[] FIRING_COLOR = {255, 30, 0};       // Hot red-orange
    public static final int[] FIRING_HIGHLIGHT = {255, 160, 0};  // Bright orange flash
    public static final int[] AIMING_COLOR = {160, 0, 255};      // Vivid purple
    public static final int[] AIMING_HIGHLIGHT = {220, 160, 255}; // Lavender-white lock flash
    public static final int[] SPOOLING_COLOR = {255, 80, 0};     // Deep charging orange
    public static final int[] SPOOLING_HIGHLIGHT = {255, 200, 100}; // Hot white-orange leading edge
    public static final int[] INTAKING_COLOR = {180, 255, 0};    // Yellow-green (distinct from pure green)
    public static final int[] INTAKING_HIGHLIGHT = {255, 255, 100}; // Bright yellow-white tip
    public static final int[] CLIMBING_COLOR = {0, 60, 255};     // Deep rich blue
    public static final int[] CLIMBING_HIGHLIGHT = {100, 180, 255}; // Sky-blue tip
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
