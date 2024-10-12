package frc.robot.subsystems.drive;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldLocations;
import frc.robot.Util;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

public class Drive extends SubsystemBase {
    //    private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
    private static final double DRIVE_BASE_WIDTH = Units.inchesToMeters(21.75); // Measured from the center of the swerve wheels
    private static final double DRIVE_BASE_LENGTH = DRIVE_BASE_WIDTH;
    private static final double DRIVE_BASE_RADIUS = Math.hypot(DRIVE_BASE_WIDTH / 2.0, DRIVE_BASE_LENGTH / 2.0);
    private static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[]{
            new Translation2d(DRIVE_BASE_WIDTH / 2.0, DRIVE_BASE_LENGTH / 2.0),
            new Translation2d(DRIVE_BASE_WIDTH / 2.0, -DRIVE_BASE_LENGTH / 2.0),
            new Translation2d(-DRIVE_BASE_WIDTH / 2.0, DRIVE_BASE_LENGTH / 2.0),
            new Translation2d(-DRIVE_BASE_WIDTH / 2.0, -DRIVE_BASE_LENGTH / 2.0)
    };
    //    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    private static final double JOYSTICK_DRIVE_DEADBAND = 0.1;
    private static final double POINT_TOWARDS_TOLERANCE = Units.degreesToRadians(3);

    public final LoggedDashboardBoolean disableDriving = new LoggedDashboardBoolean("Disable Driving", false);
    private final LoggedDashboardNumber maxLinearSpeed = new LoggedDashboardNumber("Max Linear Speed (m/sec)", Units.feetToMeters(15));
    private final LoggedDashboardNumber maxAngularSpeed = new LoggedDashboardNumber("Max Angular Speed (deg/sec)", 270);
    public final LoggedDashboardBoolean disableVision = new LoggedDashboardBoolean("Disable Vision", false);

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final VisionIO visionIO;
    private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

    /**
     * FL, FR, BL, BR
     */
    private final Module[] modules = new Module[4];
    private final SysIdRoutine sysId;
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
    // For delta tracking
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    private final PIDController choreoFeedbackX = new PIDController(1, 0, 0);
    private final PIDController choreoFeedbackY = new PIDController(1, 0, 0);
    private final PIDController choreoFeedbackTheta = new PIDController(1, 0, 0);
    private final PIDController pointTowardsController = new PIDController(2, 0, 0.1);

    private static Drive instance;

    public static Drive get() {
        return instance;
    }

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            VisionIO visionIO
    ) {
        if (instance != null)
            throw new RuntimeException("Duplicate subsystem created!");
        instance = this;

        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);
        this.visionIO = visionIO;

        choreoFeedbackTheta.enableContinuousInput(-Math.PI, Math.PI);
        pointTowardsController.enableContinuousInput(-Math.PI, Math.PI);

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity,
                new HolonomicPathFollowerConfig(maxLinearSpeed.get(), DRIVE_BASE_RADIUS, new ReplanningConfig()),
                Util::shouldFlip,
                this
        );
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> Logger.recordOutput("Drive/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> Logger.recordOutput("Drive/TrajectorySetpoint", targetPose));

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            for (int i = 0; i < 4; i++) {
                                modules[i].runCharacterization(voltage.in(Volts));
                            }
                        },
                        null,
                        this
                )
        );
    }

    public void periodic() {
        visionIO.updateInputs(visionInputs);
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Inputs/Drive/Gyro", gyroInputs);
        Logger.processInputs("Inputs/Drive/Vision", visionInputs);
        for (var module : modules) {
            module.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Read wheel positions and deltas from each module
        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle
            );
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        // Update gyro angle
        if (gyroInputs.isConnected) {
            // Use the real gyro angle
            rawGyroRotation = new Rotation2d(gyroInputs.yawPositionRad);
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Apply odometry update
        poseEstimator.update(rawGyroRotation, modulePositions);

        if (!disableVision.get() && visionInputs.hasEstimatedPose) {
            double estimateDifference = visionInputs.estimatedPose.toPose2d().getTranslation().getDistance(
                    poseEstimator.getEstimatedPosition().getTranslation()
            );

            double xyStdDev;
            double rotStdDev;

            if (visionInputs.bestTargetArea > 0.8 && estimateDifference < 0.5) {
                xyStdDev = 1.0;
                rotStdDev = 10.0;
            } else if (visionInputs.bestTargetArea > 0.8) {
                xyStdDev = 1.5;
                rotStdDev = 10.0;
            } else if (visionInputs.bestTargetArea > 0.5 && estimateDifference < 1) {
                xyStdDev = 2.0;
                rotStdDev = 15.0;
            } else if (visionInputs.bestTargetArea > 0.2 && estimateDifference < 2) {
                xyStdDev = 4.0;
                rotStdDev = 30.0;
            } else if (visionInputs.bestTargetArea > 0.05 && estimateDifference < 5) {
                xyStdDev = 10.0;
                rotStdDev = 30.0;
            } else {
                xyStdDev = 30.0;
                rotStdDev = 90.0;
            }

            if (visionInputs.numTargets >= 2) {
                xyStdDev -= visionInputs.bestTargetArea > 0.8 ? 0.25 : 0.5;
                rotStdDev -= 8.0;
            }

            poseEstimator.addVisionMeasurement(
                    //new Pose2d(visionInputs.estimatedPose.toPose2d().getTranslation(), rawGyroRotation),
                    visionInputs.estimatedPose.toPose2d(),
                    visionInputs.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(rotStdDev))
            );
        }
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeed.get());

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }

        // Log setpoint states
        Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    private void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = MODULE_TRANSLATIONS[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "Drive/SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Drive/Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current odometry pose.
     */
    private void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    @AutoLogOutput(key = "Drive/SpeakerDistance")
    public Measure<Distance> distanceToSpeaker() {
        return Meters.of(getPose().getTranslation().getDistance(FieldLocations.SPEAKER.get()));
    }

    /**
     * Returns the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /**
     * Resets the current odometry pose.
     */
    public Command setPose(Supplier<Pose2d> pose) {
        return Commands.runOnce(() -> setPose(pose.get()));
    }

    private void resetRotation() {
        setPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp  The timestamp of the vision measurement in seconds.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    public Command stopWithXCommand() {
        return runOnce(this::stopWithX);
    }

    public Command resetRotationCommand() {
        return runOnce(() -> get().resetRotation()).ignoringDisable(true);
    }

    public Command driveVelocity(ChassisSpeeds velocities, double seconds) {
        return run(() -> runVelocity(velocities)).withTimeout(seconds);
    }

    public AutoFactory createAutoFactory() {
        return Choreo.createAutoFactory(
                this,
                this::getPose,
                this::choreoController,
                Util::shouldFlip,
                new AutoFactory.AutoBindings()
        );
    }

    private void choreoController(Pose2d pose, SwerveSample sample) {
        runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                sample.vx + choreoFeedbackX.calculate(pose.getX(), sample.x),
                sample.vy + choreoFeedbackY.calculate(pose.getY(), sample.y),
                sample.omega + choreoFeedbackTheta.calculate(pose.getRotation().getRadians(), sample.heading),
                pose.getRotation()
        ));
    }

    private void runDrive(double linearMagnitude, Rotation2d linearDirection, double omega) {
        // Calculate new linear velocity
        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();

        // Convert to field relative speeds & send command
        runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * maxLinearSpeed.get(),
                        linearVelocity.getY() * maxLinearSpeed.get(),
                        omega * Units.degreesToRadians(maxAngularSpeed.get()),
                        Util.shouldFlip()
                                ? getRotation()
                                : getRotation().plus(new Rotation2d(Math.PI))
                )
        );
    }

    private double calculateLinearMagnitude(double x, double y) {
        var linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), JOYSTICK_DRIVE_DEADBAND);
        return linearMagnitude * linearMagnitude;
    }

    private Rotation2d calculateLinearDirection(double x, double y) {
        return new Rotation2d(x, y);
    }

    private double calculateOmega(double omega) {
        double omegaWithDeadband = MathUtil.applyDeadband(omega, JOYSTICK_DRIVE_DEADBAND);
        return Math.copySign(omegaWithDeadband * omegaWithDeadband, omegaWithDeadband);
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public Command driveJoystick(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return run(() -> {
            var x = xSupplier.getAsDouble();
            var y = ySupplier.getAsDouble();
            var omega = omegaSupplier.getAsDouble();

            runDrive(
                    calculateLinearMagnitude(x, y),
                    calculateLinearDirection(x, y),
                    calculateOmega(omega)
            );
        });
    }

    public Command driveJoystickPointShooterTowards(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Translation2d> pointToPointTowards) {
        var cmd = run(() -> {
            var x = xSupplier.getAsDouble();
            var y = ySupplier.getAsDouble();
            var point = pointToPointTowards.get();

            var angleTowards = Util.angle(point, getPose().getTranslation());
            Logger.recordOutput("Drive/PointTowards/Setpoint", angleTowards);
            var omega = pointTowardsController.calculate(getRotation().getRadians() + Math.PI, angleTowards);

            runDrive(
                    calculateLinearMagnitude(x, y),
                    calculateLinearDirection(x, y),
                    omega
            );
        });
        return new WrapperCommand(cmd) {
            @Override
            public void initialize() {
                super.initialize();
                Logger.recordOutput("Drive/PointTowards/Running", true);
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                Logger.recordOutput("Drive/PointTowards/Running", false);
            }
        };
    }

    public boolean pointingShooterTowardsPoint(Translation2d pointTowards) {
        var angleTowards = Util.angle(pointTowards, getPose().getTranslation());
        return Math.abs(MathUtil.angleModulus(getRotation().getRadians() + Math.PI) - angleTowards) <= POINT_TOWARDS_TOLERANCE;
    }
}
