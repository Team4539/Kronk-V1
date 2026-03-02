package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.DashboardHelper;
import frc.robot.util.DashboardHelper.Category;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.GameStateManager;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.ShootingCalculator;
@SuppressWarnings("unused")

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    // FIELD VISUALIZATION - Primary field widget for all robot visualization
    
    /** Main field widget - shows robot pose, targets, aim lines, etc. */
    private final Field2d m_field2d = new Field2d();
    
    /** Robot object on the field */
    private final FieldObject2d m_robotObject;
    
    /** Hub target marker */
    private final FieldObject2d m_hubObject;
    
    /** Trench rotating target marker */
    private final FieldObject2d m_trenchRotatingObject;
    
    /** Trench fixed target marker */
    private final FieldObject2d m_trenchFixedObject;
    
    /** Current aim target (hub or trench) */
    private final FieldObject2d m_aimTargetObject;
    
    /** Aim line from robot to target */
    private final FieldObject2d m_aimLineObject;
    
    /** Auto-shuttle boundary line */
    private final FieldObject2d m_shuttleBoundaryObject;

    /** Computed target point (where ShootingCalculator says the shooter is aiming) */
    private final FieldObject2d m_piTargetObject;

    /** Computed aim line (from robot to ShootingCalculator's calculated target) */
    private final FieldObject2d m_piAimLineObject;

    // FIELD INITIALIZATION HELPER - Initializes field objects (must be called in constructor)
    
    {
        // Instance initializer - runs before constructors
        m_robotObject = m_field2d.getObject("Robot");
        m_hubObject = m_field2d.getObject("Hub Target");
        m_trenchRotatingObject = m_field2d.getObject("Trench Rotating");
        m_trenchFixedObject = m_field2d.getObject("Trench Fixed");
        m_aimTargetObject = m_field2d.getObject("Current Target");
        m_aimLineObject = m_field2d.getObject("Aim Line");
        m_shuttleBoundaryObject = m_field2d.getObject("Shuttle Boundary");
        m_piTargetObject = m_field2d.getObject("Calc Target");
        m_piAimLineObject = m_field2d.getObject("Calc Aim Line");
        
        // Publish to SmartDashboard at top-level "Field"
        DashboardHelper.putData(Category.MATCH, "Field", m_field2d);
    }

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second^2, but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]^T, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]^T, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Flip path for Red alliance (paths are drawn for Blue alliance origin)
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Blue
                        ? kBlueAlliancePerspectiveRotation
                        : kRedAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        
        // Update field visualization
        updateFieldVisualization();
    }
    
    /**
     * Updates the Field2d widget with current robot pose and targets.
     * This is called every periodic() cycle.
     */
    private void updateFieldVisualization() {
        // Get current pose from odometry
        Pose2d currentPose = getState().Pose;
        
        // Update robot pose on the field (primary visualization)
        m_field2d.setRobotPose(currentPose);
        m_robotObject.setPose(currentPose);
        
        // Get alliance for target positions
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        
        // Get target positions based on alliance
        var hubPosition = (alliance == Alliance.Blue) ? 
            Constants.Field.BLUE_HUB_CENTER : Constants.Field.RED_HUB_CENTER;
        var trenchRotating = (alliance == Alliance.Blue) ? 
            Constants.Field.BLUE_TRENCH_ROTATING : Constants.Field.RED_TRENCH_ROTATING;
        var trenchFixed = (alliance == Alliance.Blue) ? 
            Constants.Field.BLUE_TRENCH_FIXED : Constants.Field.RED_TRENCH_FIXED;
        
        // Update target markers
        m_hubObject.setPose(new Pose2d(hubPosition, Rotation2d.kZero));
        m_trenchRotatingObject.setPose(new Pose2d(trenchRotating, Rotation2d.kZero));
        m_trenchFixedObject.setPose(new Pose2d(trenchFixed, Rotation2d.kZero));
        
        // Determine current target (hub or trench based on shuttle mode)
        GameStateManager gameState = GameStateManager.getInstance();
        boolean isShuttleMode = gameState.isShuttleMode();
        
        var currentTarget = isShuttleMode ? 
            getClosestTrench(currentPose, trenchRotating, trenchFixed) : hubPosition;
        
        m_aimTargetObject.setPose(new Pose2d(currentTarget, Rotation2d.kZero));
        
        // Draw aim line from robot center to target (fixed shooter, no turret offset)
        Translation2d robotPosition = currentPose.getTranslation();
        drawAimLine(robotPosition, currentTarget);
        
        // Draw ShootingCalculator aim line using actual shooting solution (robot heading + distance)
        drawCalculatedAimLine(currentPose, robotPosition);
        
        // Draw shuttle boundary line
        drawShuttleBoundaryLine(alliance);
    }
    
    /**
     * Gets the closest trench target to the robot's current position.
     */
    private edu.wpi.first.math.geometry.Translation2d getClosestTrench(
            Pose2d robotPose, 
            edu.wpi.first.math.geometry.Translation2d rotating, 
            edu.wpi.first.math.geometry.Translation2d fixed) {
        double distToRotating = robotPose.getTranslation().getDistance(rotating);
        double distToFixed = robotPose.getTranslation().getDistance(fixed);
        return (distToRotating <= distToFixed) ? rotating : fixed;
    }
    
    /**
     * Draws the aim line from robot to target on the field.
     */
    private void drawAimLine(edu.wpi.first.math.geometry.Translation2d from, 
                             edu.wpi.first.math.geometry.Translation2d to) {
        int numPoints = 10;
        Pose2d[] linePoints = new Pose2d[numPoints];
        
        double angle = Math.toDegrees(Math.atan2(to.getY() - from.getY(), to.getX() - from.getX()));
        
        for (int i = 0; i < numPoints; i++) {
            double t = (double) i / (numPoints - 1);
            double x = from.getX() + t * (to.getX() - from.getX());
            double y = from.getY() + t * (to.getY() - from.getY());
            linePoints[i] = new Pose2d(x, y, Rotation2d.fromDegrees(angle));
        }
        
        m_aimLineObject.setPoses(linePoints);
    }

    /**
     * Draws the ShootingCalculator's computed aim line on the field.
     * Uses the calculator's angle-to-target and distance to show where
     * the fixed shooter is pointed, based on robot heading.
     * 
     * @param robotPose Current robot pose
     * @param shooterPosition Shooter position in field coordinates (robot center)
     */
    private void drawCalculatedAimLine(Pose2d robotPose, Translation2d shooterPosition) {
        ShootingCalculator calc = ShootingCalculator.getInstance();
        
        double angleToTarget = calc.getAngleToTarget();  // degrees, robot-relative
        double distance = calc.getDistance();              // meters to target
        
        // If distance is zero or very small, nothing to draw
        if (distance < 0.1) {
            m_piAimLineObject.setPoses(new Pose2d[0]);
            m_piTargetObject.setPoses(new Pose2d[0]);
            return;
        }
        
        // Convert robot-relative angle to field angle
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        double fieldAngleDeg = robotHeadingDeg + angleToTarget;
        double fieldAngleRad = Math.toRadians(fieldAngleDeg);
        
        // Calculate the computed target point
        double targetX = shooterPosition.getX() + distance * Math.cos(fieldAngleRad);
        double targetY = shooterPosition.getY() + distance * Math.sin(fieldAngleRad);
        Translation2d computedTarget = new Translation2d(targetX, targetY);
        
        // Set target marker
        m_piTargetObject.setPose(new Pose2d(computedTarget, Rotation2d.fromDegrees(fieldAngleDeg)));
        
        // Draw line from shooter to target point
        int numPoints = 10;
        Pose2d[] linePoints = new Pose2d[numPoints];
        Rotation2d lineRotation = Rotation2d.fromDegrees(fieldAngleDeg);
        
        for (int i = 0; i < numPoints; i++) {
            double t = (double) i / (numPoints - 1);
            double x = shooterPosition.getX() + t * (targetX - shooterPosition.getX());
            double y = shooterPosition.getY() + t * (targetY - shooterPosition.getY());
            linePoints[i] = new Pose2d(x, y, lineRotation);
        }
        
        m_piAimLineObject.setPoses(linePoints);
        
        // Publish aim data to dashboard for debugging
        DashboardHelper.putNumber(Category.DEBUG, "Aim/FieldAngleDeg", fieldAngleDeg);
        DashboardHelper.putNumber(Category.DEBUG, "Aim/Distance", distance);
        DashboardHelper.putNumber(Category.DEBUG, "Aim/AngleToTarget", angleToTarget);
        DashboardHelper.putNumber(Category.DEBUG, "Aim/TargetX", targetX);
        DashboardHelper.putNumber(Category.DEBUG, "Aim/TargetY", targetY);
        DashboardHelper.putBoolean(Category.DEBUG, "Aim/IsMoving", calc.isMoving());
    }
    
    /**
     * Draws the auto-shuttle boundary line on the field.
     */
    private void drawShuttleBoundaryLine(Alliance alliance) {
        double boundaryX = Constants.Field.AUTO_SHUTTLE_BOUNDARY_X;
        
        // For Red alliance, mirror the boundary line
        if (alliance == Alliance.Red) {
            boundaryX = Constants.Field.FIELD_LENGTH_METERS - boundaryX;
        }
        
        // Create vertical line from bottom to top of field
        int numPoints = 5;
        Pose2d[] linePoints = new Pose2d[numPoints];
        
        for (int i = 0; i < numPoints; i++) {
            double t = (double) i / (numPoints - 1);
            double y = t * Constants.Field.FIELD_WIDTH_METERS;
            linePoints[i] = new Pose2d(boundaryX, y, Rotation2d.fromDegrees(90));
        }
        
        m_shuttleBoundaryObject.setPoses(linePoints);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]^T, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
    
    // GETTERS
    
    /**
     * Gets the Field2d widget for visualization.
     * This is the main field widget published to SmartDashboard as "Field".
     * @return The Field2d widget
     */
    public Field2d getField2d() {
        return m_field2d;
    }
    
    /**
     * Gets the current robot pose from odometry.
     * @return Current robot pose
     */
    public Pose2d getPose() {
        return getState().Pose;
    }
}

