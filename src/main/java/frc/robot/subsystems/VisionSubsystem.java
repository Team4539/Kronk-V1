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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;

/**
 * Handles vision processing using a single PhotonVision front camera for pose estimation.
 * 
 * CAMERA LAYOUT:
 *   - FRONT camera: mounted up top, pointing forward — sees scoring-side tags
 * 
 * The camera provides AprilTag pose estimates that are fused into the
 * drivetrain's pose estimator. Each cycle, the camera is polled and its
 * result is used as the subsystem's canonical pose for aiming calculations.
 */
public class VisionSubsystem extends SubsystemBase {

    // =========================================================================
    // INNER CLASS: CameraModule — encapsulates one camera + its pose estimator
    // =========================================================================

    /**
     * Wraps a single PhotonVision camera and its associated pose estimator.
     * Handles per-camera result processing, ambiguity filtering, and pose extraction.
     * 
     * All camera communication is wrapped in try-catch so that a disconnected
     * or erroring camera never takes down the other camera or the subsystem.
     */
    private static class CameraModule {
        final String label;
        final PhotonCamera camera;
        final PhotonPoseEstimator poseEstimator;

        // Per-frame results
        Pose2d pose = new Pose2d();
        double timestamp = 0;
        boolean hasPose = false;
        boolean hasTarget = false;
        int targetCount = 0;
        int bestTargetId = -1;
        double bestAmbiguity = 1.0;
        double horizontalOffset = 0.0;
        double targetArea = 0.0;
        boolean isMultiTag = false;
        String poseMethod = "None";

        // Error tracking — avoids spamming console on persistent failure
        int consecutiveErrors = 0;
        private static final int ERROR_LOG_INTERVAL = 250; // ~5 seconds at 50Hz

        CameraModule(String label, PhotonCamera camera, Transform3d robotToCamera,
                     AprilTagFieldLayout fieldLayout) {
            this.label = label;
            this.camera = camera;
            this.poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
        }

        /** Feed gyro heading to the pose estimator for single-tag strategies. */
        void addHeadingData(double timestampSec, Rotation2d heading) {
            try {
                poseEstimator.addHeadingData(timestampSec, heading);
            } catch (Exception e) {
                // Non-critical — will just fall back to LowestAmbiguity strategy
            }
        }

        /**
         * Process the latest results from this camera.
         * If the camera throws any exception (disconnected, NT timeout, etc.)
         * we clear the pose and return false — the other camera continues working.
         * @param headingSeeded whether the gyro heading has been initialized from vision
         * @return true if a valid pose was produced this cycle
         */
        boolean update(boolean headingSeeded) {
            try {
                return updateInternal(headingSeeded);
            } catch (Exception e) {
                consecutiveErrors++;
                if (consecutiveErrors == 1 || consecutiveErrors % ERROR_LOG_INTERVAL == 0) {
                    System.err.println("[Vision] " + label + " camera error (#"
                            + consecutiveErrors + "): " + e.getMessage());
                }
                clearPose();
                return false;
            }
        }

        private boolean updateInternal(boolean headingSeeded) {
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            if (results.isEmpty()) {
                // No new frames — keep previous hasPose state (avoid flicker)
                return hasPose;
            }

            PhotonPipelineResult latest = results.get(results.size() - 1);
            hasTarget = latest.hasTargets();

            if (!hasTarget) {
                clearPose();
                return false;
            }

            // Cache best-target info
            PhotonTrackedTarget best = latest.getBestTarget();
            if (best != null) {
                bestTargetId = best.getFiducialId();
                horizontalOffset = best.getYaw();
                targetArea = best.getArea();
                bestAmbiguity = best.getPoseAmbiguity();
            }
            targetCount = latest.getTargets().size();

            // Reject high-ambiguity single-tag results
            if (targetCount == 1 && bestAmbiguity > Constants.Vision.MAX_AMBIGUITY) {
                hasPose = false;
                isMultiTag = false;
                poseMethod = "REJECTED(ambiguity " + String.format("%.3f", bestAmbiguity) + ")";
                return false;
            }

            // --- Try multi-tag first (best accuracy) ---
            Optional<EstimatedRobotPose> est = poseEstimator.estimateCoprocMultiTagPose(latest);
            poseMethod = "MultiTag";
            isMultiTag = est.isPresent();

            if (est.isEmpty()) {
                isMultiTag = false;
                if (headingSeeded) {
                    est = poseEstimator.estimatePnpDistanceTrigSolvePose(latest);
                    poseMethod = "PnpDistanceTrig";
                } else {
                    est = poseEstimator.estimateLowestAmbiguityPose(latest);
                    poseMethod = "LowestAmbiguity(seeding)";
                }
            }

            if (est.isEmpty()) {
                // Last resort fallback
                if (headingSeeded) {
                    est = poseEstimator.estimateLowestAmbiguityPose(latest);
                    poseMethod = "LowestAmbiguity(fallback)";
                } else {
                    est = poseEstimator.estimatePnpDistanceTrigSolvePose(latest);
                    poseMethod = "PnpDistanceTrig(fallback)";
                }
            }

            if (est.isPresent()) {
                EstimatedRobotPose erp = est.get();
                Pose2d candidate = erp.estimatedPose.toPose2d();

                // Reject poses outside the field
                double margin = 0.5;
                if (candidate.getX() < -margin
                        || candidate.getX() > Constants.Field.FIELD_LENGTH_METERS + margin
                        || candidate.getY() < -margin
                        || candidate.getY() > Constants.Field.FIELD_WIDTH_METERS + margin) {
                    hasPose = false;
                    poseMethod = "REJECTED(off-field)";
                    return false;
                }

                pose = candidate;
                timestamp = erp.timestampSeconds;
                hasPose = true;
                consecutiveErrors = 0; // Camera is working — reset error counter
                return true;
            }

            hasPose = false;
            return false;
        }

        private void clearPose() {
            hasPose = false;
            bestTargetId = -1;
            horizontalOffset = 0.0;
            targetArea = 0.0;
            targetCount = 0;
            bestAmbiguity = 1.0;
            isMultiTag = false;
            poseMethod = "None";
        }

        boolean isConnected() {
            try {
                return camera.isConnected();
            } catch (Exception e) {
                return false;
            }
        }
    }

    // =========================================================================
    // FIELDS
    // =========================================================================

    private final AprilTagFieldLayout fieldLayout;

    // Camera module
    private final CameraModule frontCam;

    // Merged "best" pose (used for aiming, distance, etc.)
    private Pose2d robotPose = new Pose2d();
    private double poseTimestamp = 0;
    private boolean hasPose = false;
    private boolean hasTargetInView = false;
    private int bestTargetId = -1;
    private double horizontalOffset = 0.0;
    private double targetArea = 0.0;
    private int targetCount = 0;
    private double bestTargetAmbiguity = 1.0;
    private boolean isMultiTagEstimate = false;

    // Drivetrain fallback
    private Pose2d drivetrainPose = new Pose2d();
    private boolean useDrivetrainFallback = true;

    // Heading seeding
    private boolean headingSeeded = false;

    private final GameStateManager gameState = GameStateManager.getInstance();
    private int telemetryCounter = 0;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================

    public VisionSubsystem() {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // --- Front camera ---
        Transform3d frontTransform = new Transform3d(
            new Translation3d(
                Constants.Vision.FRONT_CAMERA_X,
                Constants.Vision.FRONT_CAMERA_Y,
                Constants.Vision.FRONT_CAMERA_Z),
            new Rotation3d(
                0.0,
                Math.toRadians(Constants.Vision.FRONT_CAMERA_PITCH_DEG),
                Math.toRadians(Constants.Vision.FRONT_CAMERA_YAW_DEG)));

        frontCam = new CameraModule(
            "Front",
            new PhotonCamera(Constants.Vision.FRONT_CAMERA_NAME),
            frontTransform,
            fieldLayout);

        initializeTunableValues();

        System.out.println("[VisionSubsystem] Single-camera init - Front: "
            + Constants.Vision.FRONT_CAMERA_NAME);
    }

    private void initializeTunableValues() {
        DashboardHelper.putNumber(Category.TUNING, "Aim/HubOffsetX", 0.0);
        DashboardHelper.putNumber(Category.TUNING, "Aim/HubOffsetY", 0.0);
        DashboardHelper.putNumber(Category.TUNING, "Aim/TrenchOffsetX", 0.0);
        DashboardHelper.putNumber(Category.TUNING, "Aim/TrenchOffsetY", 0.0);
    }

    // =========================================================================
    // DIAGNOSTIC
    // =========================================================================

    /** Check if the camera is connected. */
    public boolean isHealthy() {
        return frontCam.isConnected();
    }

    /** Check if the front camera is connected. */
    public boolean isFrontCameraConnected() {
        return frontCam.isConnected();
    }

    // =========================================================================
    // TARGET DETECTION (merged best result)
    // =========================================================================

    public boolean hasTarget() { return hasTargetInView; }
    public int getTargetId() { return bestTargetId; }
    public double getHorizontalOffset() { return horizontalOffset; }
    public double getTargetArea() { return targetArea; }
    public int getTargetCount() { return targetCount; }
    public boolean isMultiTag() { return isMultiTagEstimate; }
    public double getBestAmbiguity() { return bestTargetAmbiguity; }

    public double getEstimatedDistanceToTag() {
        if (hasPoseEstimate()) return getDistanceToHub();
        double area = getTargetArea();
        if (area <= 0) return 0.0;
        return 3.0 / Math.sqrt(area);
    }

    // =========================================================================
    // POSE ESTIMATION — merged & per-camera accessors
    // =========================================================================

    public boolean hasPoseEstimate() {
        if (useDrivetrainFallback) return true;
        return hasPose && hasTargetInView;
    }

    public boolean hasVisionPose() {
        return hasPose && hasTargetInView;
    }

    /** Whether the FRONT camera produced a valid pose this cycle. */
    public boolean hasFrontPose() { return frontCam.hasPose; }

    /** Front camera pose (only valid if hasFrontPose()). */
    public Pose2d getFrontPose() { return frontCam.pose; }
    /** Front camera timestamp. */
    public double getFrontTimestamp() { return frontCam.timestamp; }
    /** Front camera multi-tag? */
    public boolean isFrontMultiTag() { return frontCam.isMultiTag; }
    /** Front camera target count. */
    public int getFrontTargetCount() { return frontCam.targetCount; }

    public void setDrivetrainPose(Pose2d pose) {
        this.drivetrainPose = pose;
    }

    /**
     * Feed the robot's gyro heading to the pose estimator.
     */
    public void setRobotOrientation(double yawDegrees, double yawRate,
                                     double pitchDegrees, double pitchRate,
                                     double rollDegrees, double rollRate) {
        double now = Timer.getFPGATimestamp();
        Rotation2d heading = Rotation2d.fromDegrees(yawDegrees);
        frontCam.addHeadingData(now, heading);
    }

    public void notifyHeadingSeeded() {
        headingSeeded = true;
        System.out.println("[VisionSubsystem] Heading seeded — using PnpDistanceTrigSolve");
    }

    /**
     * Get the best available robot pose (vision or drivetrain fallback).
     */
    public Pose2d getRobotPose() {
        if (hasPose && hasTargetInView) return robotPose;
        if (useDrivetrainFallback) return drivetrainPose;
        return robotPose;
    }

    public double getPoseTimestamp() { return poseTimestamp; }

    public Translation2d getShooterFieldPosition() {
        return getRobotPose().getTranslation();
    }

    // =========================================================================
    // DISTANCE & ANGLE TO HUB
    // =========================================================================

    public double getDistanceToHub() {
        return getShooterFieldPosition().getDistance(getHubPosition());
    }

    public double getAngleToHub() {
        Translation2d hub = getHubPosition();
        Translation2d robot = getShooterFieldPosition();
        return Math.toDegrees(Math.atan2(hub.getY() - robot.getY(), hub.getX() - robot.getX()));
    }

    public double getRobotAngleToHub() {
        double angle = getAngleToHub() - getRobotPose().getRotation().getDegrees();
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    public Translation2d getHubPosition() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d base = (alliance == Alliance.Blue)
            ? Constants.Field.BLUE_HUB_CENTER : Constants.Field.RED_HUB_CENTER;
        double ox = DashboardHelper.getNumber(Category.TUNING, "Aim/HubOffsetX", 0.0);
        double oy = DashboardHelper.getNumber(Category.TUNING, "Aim/HubOffsetY", 0.0);
        return new Translation2d(base.getX() + ox, base.getY() + oy);
    }

    public Translation2d getHubPositionRaw() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (alliance == Alliance.Blue)
            ? Constants.Field.BLUE_HUB_CENTER : Constants.Field.RED_HUB_CENTER;
    }

    // =========================================================================
    // DISTANCE & ANGLE TO TRENCH
    // =========================================================================

    public Translation2d getTrenchPosition() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d robot = getRobotPose().getTranslation();

        Translation2d rotating, fixed;
        if (alliance == Alliance.Blue) {
            rotating = Constants.Field.BLUE_TRENCH_ROTATING;
            fixed = Constants.Field.BLUE_TRENCH_FIXED;
        } else {
            rotating = Constants.Field.RED_TRENCH_ROTATING;
            fixed = Constants.Field.RED_TRENCH_FIXED;
        }

        Translation2d base = (robot.getDistance(rotating) <= robot.getDistance(fixed))
            ? rotating : fixed;

        double ox = DashboardHelper.getNumber(Category.TUNING, "Aim/TrenchOffsetX", 0.0);
        double oy = DashboardHelper.getNumber(Category.TUNING, "Aim/TrenchOffsetY", 0.0);
        return new Translation2d(base.getX() + ox, base.getY() + oy);
    }

    public Translation2d getTrenchRotatingPosition() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (alliance == Alliance.Blue)
            ? Constants.Field.BLUE_TRENCH_ROTATING : Constants.Field.RED_TRENCH_ROTATING;
    }

    public Translation2d getTrenchFixedPosition() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (alliance == Alliance.Blue)
            ? Constants.Field.BLUE_TRENCH_FIXED : Constants.Field.RED_TRENCH_FIXED;
    }

    public double getDistanceToTrench() {
        return getShooterFieldPosition().getDistance(getTrenchPosition());
    }

    public double getAngleToTrench() {
        Translation2d trench = getTrenchPosition();
        Translation2d robot = getShooterFieldPosition();
        return Math.toDegrees(Math.atan2(trench.getY() - robot.getY(), trench.getX() - robot.getX()));
    }

    public double getRobotAngleToTrench() {
        double angle = getAngleToTrench() - getRobotPose().getRotation().getDegrees();
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    // =========================================================================
    // VISION STD DEVS
    // =========================================================================

    /**
     * Get standard deviations for the merged best vision measurement.
     */
    public double[] getVisionStdDevs() {
        double distance = getDistanceToHub();
        double scaleFactor = Math.max(1.0, distance / 3.0);
        double mtMultiplier = (targetCount >= 2) ? 1.0 : 1.5;
        return new double[] {
            Constants.Vision.VISION_STD_DEV_X * scaleFactor * mtMultiplier,
            Constants.Vision.VISION_STD_DEV_Y * scaleFactor * mtMultiplier,
            Constants.Vision.VISION_STD_DEV_THETA * scaleFactor * mtMultiplier
        };
    }

    /**
     * Get std devs for a specific camera's result (used by RobotContainer for
     * independent per-camera fusion).
     */
    public double[] getStdDevsForCamera(boolean camIsMultiTag, int camTargetCount) {
        double distance = getDistanceToHub();
        double scaleFactor = Math.max(1.0, distance / 3.0);
        double mtMultiplier = (camTargetCount >= 2) ? 1.0 : 1.5;
        return new double[] {
            Constants.Vision.VISION_STD_DEV_X * scaleFactor * mtMultiplier,
            Constants.Vision.VISION_STD_DEV_Y * scaleFactor * mtMultiplier,
            Constants.Vision.VISION_STD_DEV_THETA * scaleFactor * mtMultiplier
        };
    }

    // =========================================================================
    // TAG SELECTION
    // =========================================================================

    public int selectBestTag(int[] visibleTags) {
        if (visibleTags.length == 0) return -1;

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

    // =========================================================================
    // PERIODIC — poll both cameras, merge best result
    // =========================================================================

    @Override
    public void periodic() {
        frontCam.update(headingSeeded);
        updateFromCamera();
        publishTelemetry();
    }

    /**
     * Updates the subsystem's canonical pose from the front camera result.
     */
    private void updateFromCamera() {
        if (frontCam.hasPose) {
            robotPose = frontCam.pose;
            poseTimestamp = frontCam.timestamp;
            hasPose = true;
            hasTargetInView = true;
            bestTargetId = frontCam.bestTargetId;
            horizontalOffset = frontCam.horizontalOffset;
            targetArea = frontCam.targetArea;
            targetCount = frontCam.targetCount;
            bestTargetAmbiguity = frontCam.bestAmbiguity;
            isMultiTagEstimate = frontCam.isMultiTag;
        } else {
            hasPose = false;
            hasTargetInView = frontCam.hasTarget;
            bestTargetId = -1;
            horizontalOffset = 0.0;
            targetArea = 0.0;
            targetCount = 0;
            bestTargetAmbiguity = 1.0;
            isMultiTagEstimate = false;
        }
    }

    // =========================================================================
    // SHUTTLE ZONE
    // =========================================================================

    public boolean isInShuttleZone() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double boundaryX = Constants.Field.AUTO_SHUTTLE_BOUNDARY_X;
        double robotX = getRobotPose().getX();
        if (alliance == Alliance.Blue) {
            return robotX > boundaryX;
        } else {
            return robotX < (Constants.Field.FIELD_LENGTH_METERS - boundaryX);
        }
    }

    // =========================================================================
    // TELEMETRY
    // =========================================================================

    private void publishTelemetry() {
        telemetryCounter++;

        // Essential — every cycle
        DashboardHelper.putBoolean(Category.MATCH, "Vision/HasTarget", hasTarget());
        DashboardHelper.putBoolean(Category.MATCH, "Vision/HasPose", hasPoseEstimate());
        DashboardHelper.putBoolean(Category.MATCH, "Vision/FrontConnected", frontCam.isConnected());

        // Pose source summary
        String poseSource;
        if (hasPose && hasTargetInView) {
            StringBuilder sb = new StringBuilder();
            if (frontCam.hasPose) {
                sb.append("F:").append(frontCam.poseMethod)
                  .append("(").append(frontCam.targetCount).append("t)");
            }
            poseSource = sb.toString().trim();
        } else if (useDrivetrainFallback) {
            poseSource = "Gyro+Odometry (No Tags)";
        } else {
            poseSource = "NONE";
        }
        DashboardHelper.putString(Category.MATCH, "Vision/PoseSource", poseSource);

        // Detailed debug telemetry every 10 cycles (~200ms)
        if (telemetryCounter >= 10) {
            telemetryCounter = 0;
            updateAimTelemetry();

            DashboardHelper.putNumber(Category.DEBUG, "Vision/TagCount", targetCount);
            DashboardHelper.putNumber(Category.DEBUG, "Vision/BestTagId", bestTargetId);
            DashboardHelper.putNumber(Category.DEBUG, "Vision/Ambiguity", bestTargetAmbiguity);

            // Per-camera debug
            DashboardHelper.putString(Category.DEBUG, "Vision/Front/Method", frontCam.poseMethod);
            DashboardHelper.putNumber(Category.DEBUG, "Vision/Front/Tags", frontCam.targetCount);
            DashboardHelper.putBoolean(Category.DEBUG, "Vision/Front/HasPose", frontCam.hasPose);
            DashboardHelper.putNumber(Category.DEBUG, "Vision/Front/Errors", frontCam.consecutiveErrors);

            // Camera health summary
            String healthStatus = frontCam.isConnected() ? "OK" : "NO CAMERA";
            DashboardHelper.putString(Category.MATCH, "Vision/CameraHealth", healthStatus);

            if (hasPose) {
                DashboardHelper.putNumber(Category.DEBUG, "Vision/RawPoseX", robotPose.getX());
                DashboardHelper.putNumber(Category.DEBUG, "Vision/RawPoseY", robotPose.getY());
                DashboardHelper.putNumber(Category.DEBUG, "Vision/RawPoseRotDeg",
                    robotPose.getRotation().getDegrees());
                DashboardHelper.putBoolean(Category.DEBUG, "Vision/IsMultiTag", isMultiTagEstimate);
            }
        }
    }

    private void updateAimTelemetry() {
        boolean isShuttleMode = gameState.isShuttleMode();
        DashboardHelper.putString(Category.MATCH, "Aim/CurrentTarget",
            isShuttleMode ? "TRENCH" : "HUB");
        DashboardHelper.putBoolean(Category.MATCH, "Aim/InShuttleZone", isInShuttleZone());
    }

    // =========================================================================
    // UTILITY
    // =========================================================================

    private boolean contains(int[] array, int value) {
        for (int item : array) {
            if (item == value) return true;
        }
        return false;
    }
}
