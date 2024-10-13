package frc.robot.subsystems.drive;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Util;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;

public class Drive extends SubsystemBase {
    public static class Dashboard {
        public static final LoggedDashboardBoolean disableDriving = new LoggedDashboardBoolean("Disable Driving", false);
        private static final LoggedDashboardNumber maxLinearSpeed = new LoggedDashboardNumber("Max Linear Speed (m/sec)", Units.feetToMeters(15));
        private static final LoggedDashboardNumber maxAngularSpeed = new LoggedDashboardNumber("Max Angular Speed (deg/sec)", 270);
        public static final LoggedDashboardBoolean disableVision = new LoggedDashboardBoolean("Disable Vision", false);
    }

    public enum State {
        IDLE,
        DRIVE_JOYSTICK,
        POINT_TOWARDS,
        FOLLOW_TRAJECTORY,
        DRIVE_VELOCITY;

        public static final State DEFAULT = State.IDLE;
    }

    private final RobotState robotState = RobotState.get();

    //    private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
    private static final double DRIVE_BASE_WIDTH = Units.inchesToMeters(21.75); // Measured from the center of the swerve wheels
    private static final double DRIVE_BASE_LENGTH = DRIVE_BASE_WIDTH;
    private static final double DRIVE_BASE_RADIUS = Math.hypot(DRIVE_BASE_WIDTH / 2.0, DRIVE_BASE_LENGTH / 2.0);
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[]{
            new Translation2d(DRIVE_BASE_WIDTH / 2.0, DRIVE_BASE_LENGTH / 2.0),
            new Translation2d(DRIVE_BASE_WIDTH / 2.0, -DRIVE_BASE_LENGTH / 2.0),
            new Translation2d(-DRIVE_BASE_WIDTH / 2.0, DRIVE_BASE_LENGTH / 2.0),
            new Translation2d(-DRIVE_BASE_WIDTH / 2.0, -DRIVE_BASE_LENGTH / 2.0)
    };
    //    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    private static final double JOYSTICK_DRIVE_DEADBAND = 0.1;
    private static final double POINT_TOWARDS_TOLERANCE = Units.degreesToRadians(3);

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final VisionIO visionIO;
    private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

    /**
     * FL, FR, BL, BR
     */
    private final Module[] modules = new Module[4];

    public final SysIdRoutine sysId;

    private final PIDController choreoFeedbackX = new PIDController(1, 0, 0);
    private final PIDController choreoFeedbackY = new PIDController(1, 0, 0);
    private final PIDController choreoFeedbackTheta = new PIDController(1, 0, 0);
    private final PIDController pointTowardsController = new PIDController(2, 0, 0.1);

    private State state = State.DEFAULT;

    private Command withState(State newState, Command command) {
        return new WrapperCommand(command) {
            @Override
            public void initialize() {
                state = newState;
                super.initialize();
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                state = State.DEFAULT;
            }
        };
    }

    private static Drive instance;

    public static Drive get() {
        if (instance == null)
            synchronized (Drive.class) {
                instance = new Drive();
            }

        return instance;
    }

    private Drive() {
        gyroIO = switch (Constants.mode) {
            case REAL -> new GyroIOPigeon2(10);
            case SIM, REPLAY -> new GyroIO();
        };
        visionIO = switch (Constants.mode) {
            case REAL -> new VisionIOCamera("Shooter_Cam");
            case SIM, REPLAY -> new VisionIO();
        };
        for (int i = 0; i < 4; i++) {
            final var moduleIO = switch (Constants.mode) {
                case REAL -> new ModuleIOSparkMaxCANcoder(i);
                case SIM -> new ModuleIOSim();
                case REPLAY -> new ModuleIO();
            };
            modules[i] = new Module(moduleIO, i);
        }

        choreoFeedbackTheta.enableContinuousInput(-Math.PI, Math.PI);
        pointTowardsController.enableContinuousInput(-Math.PI, Math.PI);

        sysId = Util.sysIdRoutine(
                "Drive",
                (voltage) -> {
                    for (int i = 0; i < 4; i++) {
                        modules[i].runCharacterization(voltage.in(Volts));
                    }
                },
                this
        );
    }

    public void periodic() {
        visionIO.updateInputs(visionInputs);
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Inputs/Drive/Gyro", gyroInputs);
        Logger.processInputs("Inputs/Drive/Vision", visionInputs);

        Logger.recordOutput("Drive/State", state);

        for (var module : modules) {
            module.periodic();
        }

        // Stop moving when idle or disabled
        if (state == State.IDLE || DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        robotState.applyOdometryUpdate(gyroInputs.isConnected ? new Rotation2d(gyroInputs.yawPositionRad) : null);

        if (!Dashboard.disableVision.get() && visionInputs.hasEstimatedPose) {
            Translation2d estimatedPosition = robotState.getPose().getTranslation();
            double estimateDifference = visionInputs.estimatedPose.toPose2d().getTranslation().getDistance(estimatedPosition);

            double xyStdDev;
            double rotStdDev;

            if (visionInputs.bestTargetArea > 0.8 && estimateDifference < 0.5) {
                xyStdDev = 1.0;
                rotStdDev = 30.0;
            } else if (visionInputs.bestTargetArea > 0.8) {
                xyStdDev = 1.5;
                rotStdDev = 35.0;
            } else if (visionInputs.bestTargetArea > 0.5 && estimateDifference < 1) {
                xyStdDev = 2.0;
                rotStdDev = 40.0;
            } else if (visionInputs.bestTargetArea > 0.2 && estimateDifference < 2) {
                xyStdDev = 4.0;
                rotStdDev = 80.0;
            } else if (visionInputs.bestTargetArea > 0.05 && estimateDifference < 5) {
                xyStdDev = 10.0;
                rotStdDev = 120.0;
            } else {
                xyStdDev = 30.0;
                rotStdDev = 360.0;
            }

            if (visionInputs.numTargets >= 2) {
                xyStdDev -= visionInputs.bestTargetArea > 0.8 ? 0.25 : 0.5;
                rotStdDev -= 8.0;
            }

            robotState.addVisionMeasurement(
                    DriverStation.isDisabled()
                            ? visionInputs.estimatedPose.toPose2d()
                            : new Pose2d(visionInputs.estimatedPose.toPose2d().getTranslation(), robotState.getRotation()),
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
        SwerveModuleState[] setpointStates = robotState.getKinematics().toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Dashboard.maxLinearSpeed.get());

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
        robotState.getKinematics().resetHeadings(headings);
        stop();
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
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    public Command driveVelocity(ChassisSpeeds velocities, double seconds) {
        var cmd = run(() -> runVelocity(velocities)).withTimeout(seconds);
        return withState(State.DRIVE_VELOCITY, cmd).withName("Drive Velocity");
    }

    public AutoFactory createAutoFactory() {
        return Choreo.createAutoFactory(
                this,
                robotState::getPose,
                this::choreoController,
                Util::shouldFlip,
                new AutoFactory.AutoBindings(),
                this::choreoTrajectoryLogger
        );
    }

    private void choreoController(Pose2d pose, SwerveSample sample) {
        Logger.recordOutput("Drive/TrajectorySetpoint", sample.getPose());
        runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                sample.vx + choreoFeedbackX.calculate(pose.getX(), sample.x),
                sample.vy + choreoFeedbackY.calculate(pose.getY(), sample.y),
                sample.omega + choreoFeedbackTheta.calculate(pose.getRotation().getRadians(), sample.heading),
                pose.getRotation()
        ));
    }

    private void choreoTrajectoryLogger(Trajectory<SwerveSample> trajectory, boolean running) {
        if (running)
            state = State.FOLLOW_TRAJECTORY;
        else
            state = State.DEFAULT;
        Logger.recordOutput("Drive/Trajectory", trajectory.getPoses());
    }

    private void runDrive(double linearMagnitude, Rotation2d linearDirection, double omega) {
        // Calculate new linear velocity
        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();

        // Convert to field relative speeds & send command
        runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * Dashboard.maxLinearSpeed.get(),
                        linearVelocity.getY() * Dashboard.maxLinearSpeed.get(),
                        omega * Units.degreesToRadians(Dashboard.maxAngularSpeed.get()),
                        Util.shouldFlip()
                                ? robotState.getRotation()
                                : robotState.getRotation().plus(new Rotation2d(Math.PI))
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
        var cmd = run(() -> {
            var x = xSupplier.getAsDouble();
            var y = ySupplier.getAsDouble();
            var omega = omegaSupplier.getAsDouble();

            runDrive(
                    calculateLinearMagnitude(x, y),
                    calculateLinearDirection(x, y),
                    calculateOmega(omega)
            );
        });
        return withState(State.DRIVE_JOYSTICK, cmd).withName("Drive Joystick");
    }

    public Command driveJoystickPointShooterTowards(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Translation2d> pointToPointTowards) {
        var cmd = run(() -> {
            var x = xSupplier.getAsDouble();
            var y = ySupplier.getAsDouble();
            var point = pointToPointTowards.get();

            var angleTowards = Util.angle(point, robotState.getTranslation());
            Logger.recordOutput("Drive/PointTowards/Setpoint", angleTowards);
            var omega = pointTowardsController.calculate(robotState.getRotation().getRadians() + Math.PI, angleTowards);

            runDrive(
                    calculateLinearMagnitude(x, y),
                    calculateLinearDirection(x, y),
                    omega
            );
        });
        return withState(State.POINT_TOWARDS, cmd).withName("Drive Joystick Point Towards");
    }

    public boolean pointingShooterTowardsPoint(Translation2d pointTowards) {
        var angleTowards = Util.angle(pointTowards, robotState.getTranslation());
        return Math.abs(MathUtil.angleModulus(robotState.getRotation().getRadians() + Math.PI) - angleTowards) <= POINT_TOWARDS_TOLERANCE;
    }
}
