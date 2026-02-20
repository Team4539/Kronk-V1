package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;

/**
 * Handles vision processing using a PhotonVision camera.
 * Detects AprilTags, estimates robot pose, and calculates distances/angles to targets.
 * 
 * This is a drop-in replacement for the old LimelightSubsystem, maintaining
 * the same public interface so all commands work without modification.
 */
public class VisionSubsystem extends SubsystemBase {

    // PhotonVision components
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;

    // Camera mounting position relative to robot center
    // This transform goes from robot center to camera position
    private final Transform3d robotToCamera;

    // Cached pose data

    /** Current robot pose estimate from vision */
    private Pose2d robotPose = new Pose2d();

    /** FPGA timestamp when pose was captured */
    private double poseTimestamp = 0;

    /** Whether we have a valid vision pose */
    private boolean hasPose = false;

    /** Whether we have any target in view */
    private boolean hasTargetInView = false;

    /** ID of the best currently tracked AprilTag */
    private int bestTargetId = -1;

    /** Horizontal offset to best target in degrees */
    private double horizontalOffset = 0.0;

    /** Target area of best target */
    private double targetArea = 0.0;

    /** Number of targets currently visible */
    private int targetCount = 0;

    /** Ambiguity of the best target (lower = more confident) */
    private double bestTargetAmbiguity = 1.0;

    /** Whether the current pose estimate came from multi-tag (2+ tags) */
    private boolean isMultiTagEstimate = false;

    /** Fallback pose from drivetrain odometry */
    private Pose2d drivetrainPose = new Pose2d();

    /** Whether to use drivetrain pose when vision unavailable */
    private boolean useDrivetrainFallback = true;

    /** Whether the gyro heading has been seeded from a vision measurement.
     *  Until this is true, we use LowestAmbiguity (PNP-derived rotation) instead of
     *  PnpDistanceTrigSolve (which just echoes back the gyro heading). */
    private boolean headingSeeded = false;

    /** Cached GameStateManager to avoid repeated getInstance() calls */
    private final GameStateManager gameState = GameStateManager.getInstance();

    /** Telemetry counter - only publish debug data every N cycles */
    private int telemetryCounter = 0;

    // Constructor

    /**
     * Creates the PhotonVision subsystem.
     * Connects to the PhotonVision camera and initializes the pose estimator.
     * 
     * NOTE: Field visualization is handled by CommandSwerveDrivetrain.
     * This subsystem only publishes vision-specific telemetry.
     */
    public VisionSubsystem() {
        // Load the AprilTag field layout for the current game
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        // Create the camera connection
        camera = new PhotonCamera(Constants.Vision.CAMERA_NAME);

        // Define camera position on robot (translation + rotation from robot center)
        robotToCamera = new Transform3d(
            new Translation3d(
                Constants.Vision.CAMERA_X_OFFSET,
                Constants.Vision.CAMERA_Y_OFFSET,
                Constants.Vision.CAMERA_Z_OFFSET
            ),
            new Rotation3d(
                0.0, // Roll
                Math.toRadians(Constants.Vision.CAMERA_PITCH_DEGREES), // Pitch
                Math.toRadians(Constants.Vision.CAMERA_YAW_DEGREES)    // Yaw
            )
        );

        // Create the pose estimator
        // Uses the 2-argument constructor (non-deprecated) with individual estimation methods
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            robotToCamera
        );

        // Initialize tunable SmartDashboard values for target positions
        initializeTunableValues();

        System.out.println("[VisionSubsystem] PhotonVision initialized - Camera: " + Constants.Vision.CAMERA_NAME);
    }

    /**
     * Initializes SmartDashboard values for tuning target positions.
     * These can be adjusted live and later copied to Constants.
     */
    private void initializeTunableValues() {
        // Hub position offsets (add to base Constants values)
        DashboardHelper.putNumber(Category.TUNING, "Aim/HubOffsetX", 0.0);
        DashboardHelper.putNumber(Category.TUNING, "Aim/HubOffsetY", 0.0);

        // Trench position offsets (add to base Constants values - applies to BOTH trenches)
        DashboardHelper.putNumber(Category.TUNING, "Aim/TrenchOffsetX", 0.0);
        DashboardHelper.putNumber(Category.TUNING, "Aim/TrenchOffsetY", 0.0);
    }

    // Diagnostic checks

    /** Check if the camera is connected and sending data. */
    public boolean isHealthy() {
        return camera.isConnected();
    }

    // TARGET DETECTION METHODS

    /**
     * Check if PhotonVision has a valid target in view.
     * @return true if an AprilTag is being tracked
     */
    public boolean hasTarget() {
        return hasTargetInView;
    }

    /**
     * Get the ID of the currently tracked AprilTag (best target).
     * @return Tag ID (1-32), or -1 if no target
     */
    public int getTargetId() {
        return bestTargetId;
    }

    /**
     * Get the horizontal offset to the target center.
     * Negative = target is to the left, Positive = target is to the right.
     * @return Horizontal offset in degrees
     */
    public double getHorizontalOffset() {
        return horizontalOffset;
    }

    /**
     * Get the target area (how much of the image the target fills).
     * Larger area generally means closer target.
     * @return Target area as percentage of image (0-100)
     */
    public double getTargetArea() {
        return targetArea;
    }

    /**
     * Estimate distance to the currently tracked AprilTag.
     * Uses the pose estimate if available, otherwise estimates from target area.
     * @return Estimated distance in meters
     */
    public double getEstimatedDistanceToTag() {
        // If we have a valid pose, use the accurate distance to hub
        if (hasPoseEstimate()) {
            return getDistanceToHub();
        }

        // Fallback: Estimate distance from target area
        double area = getTargetArea();
        if (area <= 0) {
            return 0.0;
        }

        double k = 3.0; // Calibration constant - adjust based on testing
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
        if (useDrivetrainFallback) {
            return true;
        }
        return hasPose && hasTargetInView;
    }

    /**
     * Check if we have a valid VISION pose estimate (ignores drivetrain fallback).
     * Use this for vision fusion to avoid feedback loops.
     * @return true if vision has provided a pose estimate and we have a target
     */
    public boolean hasVisionPose() {
        return hasPose && hasTargetInView;
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
     * Feed the robot's gyro heading to the pose estimator for better single-tag estimates.
     * PhotonVision's multi-tag PNP doesn't need this, but we store it for potential
     * future use with reference pose strategies.
     * 
     * Call this every cycle from RobotContainer.updateVisionPose().
     * 
     * @param yawDegrees   Robot yaw (heading) in degrees (gyro reading)
     * @param yawRate      Robot yaw rate in degrees per second
     * @param pitchDegrees Robot pitch in degrees (0 if not available)
     * @param pitchRate    Robot pitch rate in degrees per second (0 if not available)
     * @param rollDegrees  Robot roll in degrees (0 if not available)
     * @param rollRate     Robot roll rate in degrees per second (0 if not available)
     */
    public void setRobotOrientation(double yawDegrees, double yawRate,
                                     double pitchDegrees, double pitchRate,
                                     double rollDegrees, double rollRate) {
        // Feed heading data to the pose estimator for PNP_DISTANCE_TRIG_SOLVE
        // and other strategies that benefit from known robot heading.
        poseEstimator.addHeadingData(
            edu.wpi.first.wpilibj.Timer.getFPGATimestamp(),
            Rotation2d.fromDegrees(yawDegrees)
        );
    }

    /**
     * Notify the vision subsystem that the gyro heading has been seeded from vision.
     * After this is called, PnpDistanceTrigSolve can be used safely because the
     * gyro heading it reads back will be correct.
     */
    public void notifyHeadingSeeded() {
        headingSeeded = true;
        System.out.println("[VisionSubsystem] Heading seeded — switching to PnpDistanceTrigSolve strategy");
    }

    /**
     * Get the best available robot pose.
     * Returns vision pose if available, otherwise drivetrain fallback.
     * @return Robot pose for aiming calculations
     */
    public Pose2d getRobotPose() {
        if (hasPose && hasTargetInView) {
            return robotPose;
        } else if (useDrivetrainFallback) {
            return drivetrainPose;
        }
        return robotPose;
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
     * Get the number of AprilTags currently visible.
     * Multi-tag (>= 2) gives significantly more reliable pose estimates.
     * @return Number of visible tags
     */
    public int getTargetCount() {
        return targetCount;
    }

    /**
     * Whether the current pose estimate came from multi-tag PNP (2+ tags).
     * Multi-tag estimates have much more reliable rotation and position.
     * @return true if current pose was computed from multiple tags
     */
    public boolean isMultiTag() {
        return isMultiTagEstimate;
    }

    /**
     * Get the ambiguity of the best currently tracked target.
     * Lower = more confident. Typically reject poses with ambiguity > 0.2.
     * @return Ambiguity value (0-1)
     */
    public double getBestAmbiguity() {
        return bestTargetAmbiguity;
    }

    /**
     * Get the turret position in field coordinates.
     * Accounts for the turret's offset from robot center.
     * @return Turret position as Translation2d in field coordinates
     */
    public Translation2d getTurretFieldPosition() {
        Pose2d robot = getRobotPose();

        double turretX = Constants.Turret.TURRET_X_OFFSET;
        double turretY = Constants.Turret.TURRET_Y_OFFSET;

        double robotHeadingRad = robot.getRotation().getRadians();
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);

        double fieldOffsetX = turretX * cos - turretY * sin;
        double fieldOffsetY = turretX * sin + turretY * cos;

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

        double dx = hubPosition.getX() - turretPosition.getX();
        double dy = hubPosition.getY() - turretPosition.getY();

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

        double turretAngle = fieldAngleToHub - robotHeading;
        turretAngle = ((turretAngle + 180) % 360 + 360) % 360 - 180;

        return turretAngle;
    }

    /**
     * Get the hub position for our alliance, including tunable offsets.
     * @return Hub center position as Translation2d (with offsets applied)
     */
    public Translation2d getHubPosition() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d basePosition = (alliance == Alliance.Blue) ?
               Constants.Field.BLUE_HUB_CENTER :
               Constants.Field.RED_HUB_CENTER;

        double offsetX = DashboardHelper.getNumber(Category.TUNING, "Aim/HubOffsetX", 0.0);
        double offsetY = DashboardHelper.getNumber(Category.TUNING, "Aim/HubOffsetY", 0.0);

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

        double distToRotating = robotPosition.getDistance(trenchRotating);
        double distToFixed = robotPosition.getDistance(trenchFixed);

        Translation2d basePosition = (distToRotating <= distToFixed) ? trenchRotating : trenchFixed;

        double offsetX = DashboardHelper.getNumber(Category.TUNING, "Aim/TrenchOffsetX", 0.0);
        double offsetY = DashboardHelper.getNumber(Category.TUNING, "Aim/TrenchOffsetY", 0.0);

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
        turretAngle = ((turretAngle + 180) % 360 + 360) % 360 - 180;

        return turretAngle;
    }

    // VISION MEASUREMENT TRUST

    /**
     * Get standard deviations for vision measurement trust.
     * We trust vision heavily — it's the primary pose source whenever tags are visible.
     * Gyro/odometry is just the fallback when no tags are in view.
     * Still scales slightly with distance (farther = slightly less precise).
     * @return Array of [x_stddev, y_stddev, theta_stddev]
     */
    public double[] getVisionStdDevs() {
        double distance = getDistanceToHub();
        // Mild distance scaling: 1.0 at ≤3m, grows slowly beyond that
        double scaleFactor = Math.max(1.0, distance / 3.0);

        // Multi-tag is a bit tighter but we trust single-tag too
        double mtMultiplier = (targetCount >= 2) ? 1.0 : 1.5;

        return new double[]{
            Constants.Vision.VISION_STD_DEV_X * scaleFactor * mtMultiplier,
            Constants.Vision.VISION_STD_DEV_Y * scaleFactor * mtMultiplier,
            Constants.Vision.VISION_STD_DEV_THETA * scaleFactor * mtMultiplier
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

        for (int tag : visibleTags) {
            if (contains(hubTags, tag)) return tag;
        }
        for (int tag : visibleTags) {
            if (contains(trenchTags, tag)) return tag;
        }

        return visibleTags[0];
    }

    // PERIODIC UPDATE

    @Override
    public void periodic() {
        updatePoseEstimate();
        publishTelemetry();
    }

    /**
     * Updates the robot pose estimate from PhotonVision data.
     * 
     * Uses MULTI_TAG_PNP_ON_COPROCESSOR as the primary strategy, which
     * computes pose from all visible tags simultaneously for best accuracy.
     * Falls back to PnpDistanceTrigSolve or LOWEST_AMBIGUITY when only one tag is visible.
     * 
     * Filters out bad estimates:
     *  - Single-tag poses with ambiguity above MAX_AMBIGUITY are rejected
     *  - Poses outside the field boundaries are rejected
     */
    private void updatePoseEstimate() {
        // Get all unread results from PhotonVision
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            // No new frames — keep last known state
            // Don't clear hasPose here to avoid flickering when camera FPS < robot loop rate
            return;
        }

        // Process the most recent result
        PhotonPipelineResult latestResult = results.get(results.size() - 1);

        // Update target detection state
        hasTargetInView = latestResult.hasTargets();

        if (!hasTargetInView) {
            hasPose = false;
            bestTargetId = -1;
            horizontalOffset = 0.0;
            targetArea = 0.0;
            targetCount = 0;
            bestTargetAmbiguity = 1.0;
            isMultiTagEstimate = false;
            return;
        }

        // Cache target info from best target
        PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
        if (bestTarget != null) {
            bestTargetId = bestTarget.getFiducialId();
            horizontalOffset = bestTarget.getYaw(); // PhotonVision yaw = horizontal offset
            targetArea = bestTarget.getArea();
            bestTargetAmbiguity = bestTarget.getPoseAmbiguity();
        }
        targetCount = latestResult.getTargets().size();

        // --- Reject single-tag estimates with high ambiguity ---
        // Multi-tag doesn't have ambiguity issues, so only filter single-tag.
        if (targetCount == 1 && bestTargetAmbiguity > Constants.Vision.MAX_AMBIGUITY) {
            hasPose = false;
            isMultiTagEstimate = false;
            DashboardHelper.putString(Category.DEBUG, "Vision/PoseMethod",
                    "REJECTED: ambiguity " + String.format("%.3f", bestTargetAmbiguity));
            return;
        }

        // Use PhotonPoseEstimator to compute robot pose from the result.
        // Try multi-tag first (coprocessor-side PNP) — always best when available.
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(latestResult);
        String poseMethod = "MultiTag";
        isMultiTagEstimate = estimatedPose.isPresent();
        
        if (estimatedPose.isEmpty()) {
            isMultiTagEstimate = false;
            if (headingSeeded) {
                // Heading is seeded correctly, so PnpDistanceTrigSolve will give
                // accurate X/Y (from tag distance) with correct rotation (from gyro).
                estimatedPose = poseEstimator.estimatePnpDistanceTrigSolvePose(latestResult);
                poseMethod = "PnpDistanceTrig";
            } else {
                // Heading NOT seeded yet — gyro is still at 0° from boot.
                // PnpDistanceTrigSolve would just echo back 0° for rotation.
                // Use LowestAmbiguity instead: it derives rotation from PNP geometry,
                // giving us a real heading we can use to seed the gyro.
                estimatedPose = poseEstimator.estimateLowestAmbiguityPose(latestResult);
                poseMethod = "LowestAmbiguity(seeding)";
            }
        }
        
        if (estimatedPose.isEmpty()) {
            // Last resort: try whichever strategy we didn't try above
            if (headingSeeded) {
                estimatedPose = poseEstimator.estimateLowestAmbiguityPose(latestResult);
                poseMethod = "LowestAmbiguity(fallback)";
            } else {
                estimatedPose = poseEstimator.estimatePnpDistanceTrigSolvePose(latestResult);
                poseMethod = "PnpDistanceTrig(fallback)";
            }
        }

        if (estimatedPose.isPresent()) {
            EstimatedRobotPose estimate = estimatedPose.get();
            Pose3d pose3d = estimate.estimatedPose;

            // Extract 2D pose for field-plane calculations
            Pose2d candidatePose = pose3d.toPose2d();
            
            // --- Reject poses outside the field ---
            double margin = 0.5; // Allow small margin for camera offset
            if (candidatePose.getX() < -margin 
                    || candidatePose.getX() > Constants.Field.FIELD_LENGTH_METERS + margin
                    || candidatePose.getY() < -margin 
                    || candidatePose.getY() > Constants.Field.FIELD_WIDTH_METERS + margin) {
                hasPose = false;
                DashboardHelper.putString(Category.DEBUG, "Vision/PoseMethod",
                        "REJECTED: off-field (" + String.format("%.1f, %.1f", 
                                candidatePose.getX(), candidatePose.getY()) + ")");
                return;
            }
            
            robotPose = candidatePose;
            poseTimestamp = estimate.timestampSeconds;
            hasPose = true;
            
            // Debug: show raw vision pose so we can diagnose rotation issues
            DashboardHelper.putNumber(Category.DEBUG, "Vision/RawPoseX", robotPose.getX());
            DashboardHelper.putNumber(Category.DEBUG, "Vision/RawPoseY", robotPose.getY());
            DashboardHelper.putNumber(Category.DEBUG, "Vision/RawPoseRotDeg", robotPose.getRotation().getDegrees());
            DashboardHelper.putString(Category.DEBUG, "Vision/PoseMethod", poseMethod);
            DashboardHelper.putBoolean(Category.DEBUG, "Vision/IsMultiTag", isMultiTagEstimate);
        } else {
            hasPose = false;
        }
    }

    // SHUTTLE ZONE CHECK

    /**
     * Check if the robot is in the shuttle zone (past the boundary line).
     * @return true if robot should be in shuttle mode based on position
     */
    public boolean isInShuttleZone() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double boundaryX = Constants.Field.AUTO_SHUTTLE_BOUNDARY_X;
        double robotX = getRobotPose().getX();

        if (alliance == Alliance.Blue) {
            return robotX > boundaryX;
        } else {
            double mirroredBoundary = Constants.Field.FIELD_LENGTH_METERS - boundaryX;
            return robotX < mirroredBoundary;
        }
    }

    // TELEMETRY

    /**
     * Publishes vision data to SmartDashboard for debugging.
     * Throttles less critical data to reduce NetworkTables traffic.
     */
    private void publishTelemetry() {
        telemetryCounter++;

        // Essential target info every cycle
        DashboardHelper.putBoolean(Category.MATCH, "Vision/HasTarget", hasTarget());
        DashboardHelper.putBoolean(Category.MATCH, "Vision/HasPose", hasPoseEstimate());
        DashboardHelper.putBoolean(Category.MATCH, "Vision/CameraConnected", camera.isConnected());

        // Show pose source so driver knows what's happening
        String poseSource;
        if (hasPose && hasTargetInView) {
            poseSource = (targetCount >= 2) ? "MultiTag (" + targetCount + " tags)" : "SingleTag (ID " + bestTargetId + ")";
        } else if (useDrivetrainFallback) {
            poseSource = "Gyro+Odometry (No Tags)";
        } else {
            poseSource = "NONE";
        }
        DashboardHelper.putString(Category.MATCH, "Vision/PoseSource", poseSource);

        // Less critical telemetry every 10 cycles (200ms)
        if (telemetryCounter >= 10) {
            telemetryCounter = 0;
            updateAimTelemetry();

            // Extra PhotonVision diagnostics
            DashboardHelper.putNumber(Category.DEBUG, "Vision/TagCount", targetCount);
            DashboardHelper.putNumber(Category.DEBUG, "Vision/BestTagId", bestTargetId);
            DashboardHelper.putNumber(Category.DEBUG, "Vision/Ambiguity", bestTargetAmbiguity);
            DashboardHelper.putNumber(Category.DEBUG, "Vision/HorizontalOffset", horizontalOffset);
        }
    }

    /**
     * Updates aim-related telemetry (target info, shuttle zone status).
     */
    private void updateAimTelemetry() {
        boolean isShuttleMode = gameState.isShuttleMode();
        String targetName = isShuttleMode ? "TRENCH" : "HUB";
        DashboardHelper.putString(Category.MATCH, "Aim/CurrentTarget", targetName);
        DashboardHelper.putBoolean(Category.MATCH, "Aim/InShuttleZone", isInShuttleZone());
    }

    // UTILITY METHODS

    /**
     * Check if an array contains a value.
     */
    private boolean contains(int[] array, int value) {
        for (int item : array) {
            if (item == value) return true;
        }
        return false;
    }
}
