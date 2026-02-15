package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameStateManager;

/**
 * Handles vision processing using a Limelight camera.
 * Detects AprilTags, estimates robot pose, and calculates distances/angles to targets.
 */
public class LimelightSubsystem extends SubsystemBase {
    
    // Network Tables entries
    
    private final NetworkTable table;
    
    /** Target ID (AprilTag ID being tracked) */
    private final NetworkTableEntry tidEntry;
    
    /** Horizontal offset to target in degrees */
    private final NetworkTableEntry txEntry;
    
    /** Has valid target (1 = yes, 0 = no) */
    private final NetworkTableEntry tvEntry;
    
    /** Target area (percentage of image) */
    private final NetworkTableEntry taEntry;
    
    /** Robot pose in Blue alliance coordinates */
    private final NetworkTableEntry botposeBlueEntry;
    
    /** Robot pose in Red alliance coordinates */
    private final NetworkTableEntry botposeRedEntry;
    
    /** Pipeline processing latency (ms) */
    private final NetworkTableEntry tlEntry;
    
    /** Capture latency (ms) */
    private final NetworkTableEntry clEntry;

    // Cached pose data
    
    /** Current robot pose estimate from vision */
    private Pose2d robotPose = new Pose2d();
    
    /** FPGA timestamp when pose was captured */
    private double poseTimestamp = 0;
    
    /** Whether we have a valid vision pose */
    private boolean hasPose = false;
    
    /** Fallback pose from drivetrain odometry */
    private Pose2d drivetrainPose = new Pose2d();
    
    /** Whether to use drivetrain pose when vision unavailable */
    private boolean useDrivetrainFallback = true;
    
    /** Cached GameStateManager to avoid repeated getInstance() calls */
    private final GameStateManager gameState = GameStateManager.getInstance();
    
    /** Telemetry counter - only publish debug data every N cycles */
    private int telemetryCounter = 0;

    // Constructor
    
    /**
     * Creates the Limelight subsystem.
     * Connects to NetworkTables and sets up field visualization.
     * 
     * NOTE: Field visualization is now handled by CommandSwerveDrivetrain's "Field" widget.
     * This subsystem only publishes Limelight-specific telemetry.
     */
    public LimelightSubsystem() {
        // Connect to Limelight NetworkTables
        table = NetworkTableInstance.getDefault().getTable(Constants.Limelight.TABLE_NAME);
        
        // Get references to all the entries we need
        tidEntry = table.getEntry("tid");
        txEntry = table.getEntry("tx");
        tvEntry = table.getEntry("tv");
        taEntry = table.getEntry("ta");
        botposeBlueEntry = table.getEntry("botpose_wpiblue");
        botposeRedEntry = table.getEntry("botpose_wpired");
        tlEntry = table.getEntry("tl");
        clEntry = table.getEntry("cl");

        // Initialize tunable SmartDashboard values for target positions
        initializeTunableValues();
    }

    /**
     * Initializes SmartDashboard values for tuning target positions.
     * These can be adjusted live and later copied to Constants.
     */
    private void initializeTunableValues() {
        // Hub position offsets (add to base Constants values)
        DashboardHelper.putNumber(Category.SETTINGS, "Aim/HubOffsetX", 0.0);
        DashboardHelper.putNumber(Category.SETTINGS, "Aim/HubOffsetY", 0.0);

        // Trench position offsets (add to base Constants values - applies to BOTH trenches)
        DashboardHelper.putNumber(Category.SETTINGS, "Aim/TrenchOffsetX", 0.0);
        DashboardHelper.putNumber(Category.SETTINGS, "Aim/TrenchOffsetY", 0.0);

        // Auto-shuttle boundary line (X position)
        DashboardHelper.putNumber(Category.SETTINGS, "Aim/AutoShuttleLineX", Constants.Field.AUTO_SHUTTLE_BOUNDARY_X);
        DashboardHelper.putBoolean(Category.SETTINGS, "Aim/AutoShuttleEnabled", Constants.Field.AUTO_SHUTTLE_ENABLED);

        // Current target mode display
        DashboardHelper.putString(Category.SETTINGS, "Aim/CurrentTarget", "HUB");

    }

    // Diagnostic checks
    /** Check if the Limelight is reachable by verifying we've received data. */
    public boolean isHealthy() {
        // Check if we've ever received a valid target detection
        // tv entry defaults to 0.0 if never written - checking the entry exists on the table
        return table.containsKey("tv");
    }


    // TARGET DETECTION METHODS
    
    /**
     * Check if Limelight has a valid target in view.
     * @return true if an AprilTag is being tracked
     */
    public boolean hasTarget() {
        return tvEntry.getDouble(0) > 0.5;
    }

    /**
     * Get the ID of the currently tracked AprilTag.
     * @return Tag ID (1-32), or -1 if no target
     */
    public int getTargetId() {
        return (int) tidEntry.getDouble(-1);
    }

    /**
     * Get the horizontal offset to the target center.
     * Negative = target is to the left, Positive = target is to the right.
     * @return Horizontal offset in degrees
     */
    public double getHorizontalOffset() {
        return txEntry.getDouble(0.0);
    }

    /**
     * Get the target area (how much of the image the target fills).
     * Larger area generally means closer target.
     * @return Target area as percentage of image (0-100)
     */
    public double getTargetArea() {
        return taEntry.getDouble(0.0);
    }

    /**
     * Estimate distance to the currently tracked AprilTag.
     * Uses the pose estimate if available, otherwise estimates from target area.
     * This is used for the turret training system.
     * @return Estimated distance in meters
     */
    public double getEstimatedDistanceToTag() {
        // If we have a valid pose, use the accurate distance to hub
        if (hasPoseEstimate()) {
            return getDistanceToHub();
        }
        
        // Fallback: Estimate distance from target area
        // This is a rough approximation - area decreases with distance squared
        // Calibrate these constants for your specific setup
        double area = getTargetArea();
        if (area <= 0) {
            return 0.0;
        }
        
        // Rough formula: distance ≈ k / sqrt(area)
        // where k is calibrated based on known measurements
        // For a standard AprilTag at ~3m, area might be ~1%
        // So k ≈ 3 * sqrt(1) = 3
        double k = 3.0;  // Calibration constant - adjust based on testing
        return k / Math.sqrt(area);
    }

    // POSE ESTIMATION METHODS
    
    /**
     * Check if we have a valid, recent pose estimate.
     * Returns true if EITHER:
     * - Vision has a valid pose AND we have a target
     * - OR drivetrain fallback is enabled (for simulation)
     * 
     * @return true if we have a usable pose for aiming
     */
    public boolean hasPoseEstimate() {
        // In simulation or when vision is down, use drivetrain fallback
        if (useDrivetrainFallback) {
            return true;  // Always have drivetrain pose available
        }
        return hasPose && hasTarget();
    }
    
    /**
     * Check if we have a valid VISION pose estimate (ignores drivetrain fallback).
     * Use this for vision fusion to avoid feedback loops.
     * @return true if vision has provided a pose estimate and we have a target
     */
    public boolean hasVisionPose() {
        return hasPose && hasTarget();
    }
    
    /**
     * Update the drivetrain pose used as fallback when vision is unavailable.
     * Call this from RobotContainer.updateVisionPose() every cycle.
     * 
     * @param pose The current drivetrain odometry pose
     */
    public void setDrivetrainPose(Pose2d pose) {
        this.drivetrainPose = pose;
    }
    
    /**
     * Get the best available robot pose.
     * Returns vision pose if available, otherwise drivetrain fallback.
     * @return Robot pose for aiming calculations
     */
    public Pose2d getRobotPose() {
        if (hasPose && hasTarget()) {
            return robotPose;  // Use vision when available
        } else if (useDrivetrainFallback) {
            return drivetrainPose;  // Fallback to drivetrain
        }
        return robotPose;  // Return last known pose
    }

    /**
     * Get the FPGA timestamp of the pose measurement.
     * Used for latency compensation when fusing with odometry.
     * @return Timestamp in seconds
     */
    public double getPoseTimestamp() {
        return poseTimestamp;
    }

    /**
     * Get the turret position in field coordinates.
     * Accounts for the turret's offset from robot center.
     * @return Turret position as Translation2d in field coordinates
     */
    public Translation2d getTurretFieldPosition() {
        Pose2d robot = getRobotPose();
        
        // Turret offset in robot coordinates (from Constants)
        double turretX = Constants.Turret.TURRET_X_OFFSET;
        double turretY = Constants.Turret.TURRET_Y_OFFSET;
        
        // Rotate turret offset by robot heading to get field-relative offset
        double robotHeadingRad = robot.getRotation().getRadians();
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);
        
        double fieldOffsetX = turretX * cos - turretY * sin;
        double fieldOffsetY = turretX * sin + turretY * cos;
        
        // Add to robot position
        return new Translation2d(
            robot.getX() + fieldOffsetX,
            robot.getY() + fieldOffsetY
        );
    }

    // DISTANCE & ANGLE TO HUB
    
    /**
     * Get straight-line distance from TURRET to the hub center.
     * Uses actual turret position, not robot center.
     * @return Distance in meters
     */
    public double getDistanceToHub() {
        Translation2d hubPosition = getHubPosition();
        Translation2d turretPosition = getTurretFieldPosition();
        return turretPosition.getDistance(hubPosition);
    }

    /**
     * Get the FIELD-RELATIVE angle from TURRET to hub.
     * 0 degrees = toward positive X axis of field.
     * @return Angle in degrees
     */
    public double getAngleToHub() {
        Translation2d hubPosition = getHubPosition();
        Translation2d turretPosition = getTurretFieldPosition();
        
        // Calculate vector from turret to hub
        double dx = hubPosition.getX() - turretPosition.getX();
        double dy = hubPosition.getY() - turretPosition.getY();
        
        // Convert to angle
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    /**
     * Get the TURRET angle needed to point at the hub.
     * This is ROBOT-RELATIVE (accounts for robot heading).
     * @return Turret angle in degrees (-180 to +180)
     */
    public double getTurretAngleToHub() {
        double fieldAngleToHub = getAngleToHub();
        double robotHeading = getRobotPose().getRotation().getDegrees();
        
        // Convert field-relative angle to robot-relative
        // The turret needs to point at (fieldAngleToHub - robotHeading) from robot forward
        double turretAngle = fieldAngleToHub - robotHeading;
        
        // Normalize to -180 to +180 using modulo for robustness
        turretAngle = ((turretAngle + 180) % 360 + 360) % 360 - 180;
        
        return turretAngle;
    }

    /**
     * Get the hub position for our alliance, including tunable offsets.
     * Offsets can be adjusted via SmartDashboard at Aim/HubOffsetX and Aim/HubOffsetY.
     * @return Hub center position as Translation2d (with offsets applied)
     */
    public Translation2d getHubPosition() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d basePosition = (alliance == Alliance.Blue) ? 
               Constants.Field.BLUE_HUB_CENTER : 
               Constants.Field.RED_HUB_CENTER;
        
        // Apply tunable offsets from SmartDashboard
        double offsetX = DashboardHelper.getNumber(Category.SETTINGS, "Aim/HubOffsetX", 0.0);
        double offsetY = DashboardHelper.getNumber(Category.SETTINGS, "Aim/HubOffsetY", 0.0);
        
        return new Translation2d(basePosition.getX() + offsetX, basePosition.getY() + offsetY);
    }

    /**
     * Get the raw hub position without offsets (for display purposes).
     * @return Base hub position from Constants
     */
    public Translation2d getHubPositionRaw() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (alliance == Alliance.Blue) ? 
               Constants.Field.BLUE_HUB_CENTER : 
               Constants.Field.RED_HUB_CENTER;
    }

    // DISTANCE & ANGLE TO TRENCH (for shuttling)
    
    /**
     * Get the CLOSEST trench target position for our alliance, including tunable offsets.
     * Automatically picks between rotating and fixed trench based on robot position.
     * Offsets can be adjusted via SmartDashboard at Aim/TrenchOffsetX and Aim/TrenchOffsetY.
     * @return Closest trench target position as Translation2d (with offsets applied)
     */
    public Translation2d getTrenchPosition() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d robotPosition = getRobotPose().getTranslation();
        
        Translation2d trenchRotating;
        Translation2d trenchFixed;
        
        if (alliance == Alliance.Blue) {
            trenchRotating = Constants.Field.BLUE_TRENCH_ROTATING;
            trenchFixed = Constants.Field.BLUE_TRENCH_FIXED;
        } else {
            trenchRotating = Constants.Field.RED_TRENCH_ROTATING;
            trenchFixed = Constants.Field.RED_TRENCH_FIXED;
        }
        
        // Return the closest trench
        double distToRotating = robotPosition.getDistance(trenchRotating);
        double distToFixed = robotPosition.getDistance(trenchFixed);
        
        Translation2d basePosition = (distToRotating <= distToFixed) ? trenchRotating : trenchFixed;

        // Apply tunable offsets from SmartDashboard
        double offsetX = DashboardHelper.getNumber(Category.SETTINGS, "Aim/TrenchOffsetX", 0.0);
        double offsetY = DashboardHelper.getNumber(Category.SETTINGS, "Aim/TrenchOffsetY", 0.0);
        
        return new Translation2d(basePosition.getX() + offsetX, basePosition.getY() + offsetY);
    }
    
    /**
     * Get the rotating arm trench position for our alliance.
     * @return Rotating trench target position as Translation2d
     */
    public Translation2d getTrenchRotatingPosition() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (alliance == Alliance.Blue) ? 
               Constants.Field.BLUE_TRENCH_ROTATING : 
               Constants.Field.RED_TRENCH_ROTATING;
    }
    
    /**
     * Get the fixed arm trench position for our alliance.
     * @return Fixed trench target position as Translation2d
     */
    public Translation2d getTrenchFixedPosition() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (alliance == Alliance.Blue) ? 
               Constants.Field.BLUE_TRENCH_FIXED : 
               Constants.Field.RED_TRENCH_FIXED;
    }

    /**
     * Get straight-line distance from TURRET to trench target.
     * Uses actual turret position, not robot center.
     * @return Distance in meters
     */
    public double getDistanceToTrench() {
        Translation2d trenchPosition = getTrenchPosition();
        Translation2d turretPosition = getTurretFieldPosition();
        return turretPosition.getDistance(trenchPosition);
    }

    /**
     * Get the FIELD-RELATIVE angle from TURRET to trench.
     * @return Angle in degrees
     */
    public double getAngleToTrench() {
        Translation2d trenchPosition = getTrenchPosition();
        Translation2d turretPosition = getTurretFieldPosition();
        
        double dx = trenchPosition.getX() - turretPosition.getX();
        double dy = trenchPosition.getY() - turretPosition.getY();
        
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    /**
     * Get the TURRET angle needed to point at the trench.
     * @return Turret angle in degrees (-180 to +180, robot-relative)
     */
    public double getTurretAngleToTrench() {
        double fieldAngleToTrench = getAngleToTrench();
        double robotHeading = getRobotPose().getRotation().getDegrees();
        
        double turretAngle = fieldAngleToTrench - robotHeading;
        
        // Normalize to -180 to +180 using modulo for robustness
        turretAngle = ((turretAngle + 180) % 360 + 360) % 360 - 180;
        
        return turretAngle;
    }

    // VISION MEASUREMENT TRUST
    
    /**
     * Get standard deviations for vision measurement trust.
     * These control how much the pose estimator trusts vision vs odometry.
     * Scales up with distance (trust vision less when far from tags).
     * @return Array of [x_stddev, y_stddev, theta_stddev]
     */
    public double[] getVisionStdDevs() {
        // Scale uncertainty with distance (farther = less trust)
        double distance = getDistanceToHub();
        double scaleFactor = Math.max(1.0, distance / 3.0);
        
        return new double[]{
            Constants.Limelight.VISION_STD_DEV_X * scaleFactor,
            Constants.Limelight.VISION_STD_DEV_Y * scaleFactor,
            Constants.Limelight.VISION_STD_DEV_THETA * scaleFactor
        };
    }

    // TAG SELECTION
    
    /**
     * Select the best tag to track from a list of visible tags.
     * Prioritizes hub tags, then trench tags, then any other tag.
     * @param visibleTags Array of visible tag IDs
     * @return Best tag ID to track, or -1 if no tags
     */
    public int selectBestTag(int[] visibleTags) {
        if (visibleTags.length == 0) {
            return -1;
        }

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        int[] hubTags = (alliance == Alliance.Blue) ? Constants.Tags.BLUE_HUB : Constants.Tags.RED_HUB;
        int[] trenchTags = (alliance == Alliance.Blue) ? Constants.Tags.BLUE_TRENCH : Constants.Tags.RED_TRENCH;

        // Priority 1: Hub tags (for scoring)
        for (int tag : visibleTags) {
            if (contains(hubTags, tag)) {
                return tag;
            }
        }

        // Priority 2: Trench tags (for shuttling)
        for (int tag : visibleTags) {
            if (contains(trenchTags, tag)) {
                return tag;
            }
        }

        // Fallback: Any visible tag
        return visibleTags[0];
    }

    // PERIODIC UPDATE
    
    @Override
    public void periodic() {
        updatePoseEstimate();
        publishTelemetry();
    }

    /**
     * Updates the robot pose estimate from Limelight data.
     * Uses alliance-appropriate botpose entry.
     */
    private void updatePoseEstimate() {
        // No pose if no target
        if (!hasTarget()) {
            hasPose = false;
            return;
        }

        // Get botpose array based on alliance
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double[] botpose;
        
        if (alliance == Alliance.Blue) {
            botpose = botposeBlueEntry.getDoubleArray(new double[7]);
        } else {
            botpose = botposeRedEntry.getDoubleArray(new double[7]);
        }
        
        // Validate pose data
        if (botpose.length < 7) {
            hasPose = false;
            return;
        }
        
        // Extract pose components: [x, y, z, roll, pitch, yaw, latency]
        double x = botpose[0];
        double y = botpose[1];
        double yaw = botpose[5];
        double totalLatency = botpose[6];  // Total latency in ms
        
        // Check for valid data (Limelight returns all zeros when no valid pose)
        if (x == 0 && y == 0 && yaw == 0) {
            hasPose = false;
            return;
        }
        
        // Create pose
        robotPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
        
        // Calculate timestamp (current time minus latency for proper fusion)
        poseTimestamp = Timer.getFPGATimestamp() - (totalLatency / 1000.0);
        
        hasPose = true;
    }

    // TELEMETRY
    
    /**
     * Publishes all Limelight data to SmartDashboard for debugging.
     * Throttles less critical data to reduce NetworkTables traffic.
     * NOTE: Field visualization is now handled by CommandSwerveDrivetrain's "Field" widget.
     */
    private void publishTelemetry() {
        telemetryCounter++;
        
        // Essential target info every cycle
        DashboardHelper.putBoolean(Category.TELEOP, "Limelight/HasTarget", hasTarget());
        DashboardHelper.putBoolean(Category.TELEOP, "Limelight/HasPose", hasPoseEstimate());
        
        // Less critical telemetry every 10 cycles (200ms)
        if (telemetryCounter >= 10) {
            telemetryCounter = 0;
            updateAimTelemetry();
        }
    }

    /**
     * Updates aim-related telemetry (target info, shuttle zone status).
     * Field visualization is now in CommandSwerveDrivetrain.
     */
    private void updateAimTelemetry() {
        boolean isShuttleMode = gameState.isShuttleMode();
        String targetName = isShuttleMode ? "TRENCH" : "HUB";
        DashboardHelper.putString(Category.SETTINGS, "Aim/CurrentTarget", targetName);
        DashboardHelper.putBoolean(Category.SETTINGS, "Aim/InShuttleZone", isInShuttleZone());
    }

    /**
     * Check if the robot is in the shuttle zone (past the boundary line).
     * @return true if robot should be in shuttle mode based on position
     */
    public boolean isInShuttleZone() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double boundaryX = DashboardHelper.getNumber(Category.SETTINGS, "Aim/AutoShuttleLineX", Constants.Field.AUTO_SHUTTLE_BOUNDARY_X);
        double robotX = getRobotPose().getX();
        
        if (alliance == Alliance.Blue) {
            // Blue alliance: shuttle zone is PAST the boundary (higher X = closer to red alliance)
            return robotX > boundaryX;
        } else {
            // Red alliance: shuttle zone is PAST the boundary (lower X = closer to blue alliance)
            double mirroredBoundary = Constants.Field.FIELD_LENGTH_METERS - boundaryX;
            return robotX < mirroredBoundary;
        }
    }

    // UTILITY METHODS
    
    /**
     * Check if an array contains a value.
     */
    private boolean contains(int[] array, int value) {
        for (int item : array) {
            if (item == value) {
                return true;
            }
        }
        return false;
    }
}

