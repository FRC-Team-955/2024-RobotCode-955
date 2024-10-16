package frc.robot.subsystems.shooter;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Util;
import frc.robot.dashboard.DashboardAngle;
import frc.robot.dashboard.DashboardSubsystem;
import frc.robot.dashboard.TuningDashboardAngle;
import frc.robot.dashboard.TuningDashboardAnglularVelocityRPM;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.GoalBasedCommandRunner;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
    public static class Dashboard {
        public static final DashboardAngle shootingSkew = new DashboardAngle(DashboardSubsystem.SHOOTER, "Shooting Skew", Degrees.zero());

        public static final TuningDashboardAnglularVelocityRPM shootConfigurableSpeed = new TuningDashboardAnglularVelocityRPM(DashboardSubsystem.SHOOTER, "Shoot Configurable Speed", RPM.zero());
        public static final TuningDashboardAngle shootConfigurableAngle = new TuningDashboardAngle(DashboardSubsystem.SHOOTER, "Shoot Configurable Angle", Degrees.zero());
    }

    // trick Java into letting us use an enum before it is defined
    private static final Goal GOAL_WAIT_FOR_INTAKE = Goal.WAIT_FOR_INTAKE;

    public enum Goal {
        CHARACTERIZATION(() -> null, () -> null, () -> null),
        HOVER(
                () -> Degrees.of(-90),
                RPM::zero,
                () -> FeedSetpoint.velocity(RPM.zero()),
                () -> Intake.get().pivotClearOfShooter() ? Optional.empty() : Optional.of(GOAL_WAIT_FOR_INTAKE)
        ),
        WAIT_FOR_INTAKE(
                () -> Degrees.of(-30),
                RPM::zero,
                () -> FeedSetpoint.velocity(RPM.zero()),
                () -> get().goal == Goal.HOVER && get().atGoal()
                        ? Optional.empty()
                        : (Intake.get().pivotClearOfShooter() ? Optional.of(Goal.HOVER) : Optional.empty())
        ),
        SHOOT_CALCULATED(
                () -> ShooterRegression.getAngle(RobotState.get().getDistanceToSpeaker()).plus(Dashboard.shootingSkew.get()),
                () -> ShooterRegression.getSpeed(RobotState.get().getDistanceToSpeaker()),
                FeedSetpoint::shoot
        ),
        SHOOT_CONFIGURABLE(
                Dashboard.shootConfigurableAngle::get,
                Dashboard.shootConfigurableSpeed::get,
                FeedSetpoint::shoot
        ),
        SHOOT_SUBWOOFER(() -> Degrees.of(-50).plus(Dashboard.shootingSkew.get()), RPM::zero, FeedSetpoint::shoot),
        AMP(() -> Degrees.of(25), () -> RPM.of(2000), FeedSetpoint::shoot),
        EJECT(() -> Degrees.of(-60), () -> RPM.of(2000), FeedSetpoint::shoot),

        HANDOFF_WAIT_FOR_INTAKE(() -> Degrees.of(-30), RPM::zero, () -> FeedSetpoint.velocity(RPM.zero())),
        HANDOFF_READY(() -> Degrees.of(-50), RPM::zero, () -> FeedSetpoint.velocity(RPM.zero())),
        HANDOFF_FEED(() -> Degrees.of(-50), RPM::zero, () -> FeedSetpoint.velocity(RPM.of(-100)));

        public static final Goal DEFAULT = Goal.HOVER;

        public final Supplier<Measure<Angle>> pivotSetpoint;
        public final Supplier<Measure<Velocity<Angle>>> flywheelsSetpoint;
        public final Supplier<FeedSetpoint> feedSetpoint;
        public final Supplier<Optional<Goal>> goalChange;

        Goal(
                Supplier<Measure<Angle>> pivotSetpoint,
                Supplier<Measure<Velocity<Angle>>> flywheelsSetpoint,
                Supplier<FeedSetpoint> feedCommand
        ) {
            this.pivotSetpoint = pivotSetpoint;
            this.flywheelsSetpoint = flywheelsSetpoint;
            this.feedSetpoint = feedCommand;
            this.goalChange = Optional::empty;
        }

        Goal(
                Supplier<Measure<Angle>> pivotSetpoint,
                Supplier<Measure<Velocity<Angle>>> flywheelsSetpoint,
                Supplier<FeedSetpoint> feedCommand,
                Supplier<Optional<Goal>> goalChange
        ) {
            this.pivotSetpoint = pivotSetpoint;
            this.flywheelsSetpoint = flywheelsSetpoint;
            this.feedSetpoint = feedCommand;
            this.goalChange = goalChange;
        }

        public static class FeedSetpoint {
            private final Type type;
            private final Measure<Velocity<Angle>> velocity;

            private enum Type {
                Velocity,
                Shoot
            }

            private FeedSetpoint(Type type, Measure<Velocity<Angle>> velocity) {
                this.type = type;
                this.velocity = velocity;
            }

            public static FeedSetpoint velocity(Measure<Velocity<Angle>> velocity) {
                return new FeedSetpoint(Type.Velocity, velocity);
            }

            public static FeedSetpoint shoot() {
                return new FeedSetpoint(Type.Shoot, null);
            }

            public boolean isShoot() {
                return type == Type.Shoot;
            }

            public void ifVelocity(Consumer<Measure<Velocity<Angle>>> velocityConsumer) {
                if (type == Type.Velocity)
                    velocityConsumer.accept(velocity);
            }
        }
    }

    protected static final ArmFeedforward PIVOT_FF = Constants.isReal ? new ArmFeedforward(0.574, 0.90, 0.65051, 0.21235) : new ArmFeedforward(0, 0.4, 0);
    protected static final PIDConstants PIVOT_PID = Constants.isReal ? new PIDConstants(0.15/*, 0.011*/) : new PIDConstants(5, 0);
    protected static final double PIVOT_GEAR_RATIO = 40;
    protected static final Measure<Angle> PIVOT_INITIAL_POSITION = Degrees.of(-90);
    private static final Measure<Angle> PIVOT_SETPOINT_TOLERANCE = Degrees.of(1.7);

    protected static final SimpleMotorFeedforward FEED_FF = Constants.isReal ? new SimpleMotorFeedforward(0, 0) : new SimpleMotorFeedforward(0, 0.058);
    protected static final PIDConstants FEED_PID = Constants.isReal ? new PIDConstants(0.1, 0.0001) : new PIDConstants(0.1, 0);
    protected static final double FEED_GEAR_RATIO = 3;
    private static final Measure<Velocity<Angle>> FEED_SETPOINT_TOLERANCE = RPM.of(10);
    private static final double FEED_BEAM_BRAKE_DEBOUNCE = 0.05;

    private static final SimpleMotorFeedforward FLYWHEEL_TOP_FF = Constants.isReal ? new SimpleMotorFeedforward(0.23795, 0.011308, 0.0069895) : new SimpleMotorFeedforward(0, 0.01);
    private static final SimpleMotorFeedforward FLYWHEEL_BOTTOM_FF = Constants.isReal ? new SimpleMotorFeedforward(0.23279, 0.0113005, 0.0064997) : FLYWHEEL_TOP_FF;
    private static final PIDConstants FLYWHEEL_TOP_PID = Constants.isReal ? new PIDConstants(0, 0) : new PIDConstants(0.05, 0);
    private static final PIDConstants FLYWHEEL_BOTTOM_PID = Constants.isReal ? new PIDConstants(0, 0) : FLYWHEEL_TOP_PID;
    protected static final double FLYWHEEL_GEAR_RATIO = 1 / 2.0;
    private static final Measure<Velocity<Angle>> FLYWHEEL_SETPOINT_TOLERANCE = RPM.of(100);

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Debouncer hasNoteDebouncer = new Debouncer(FEED_BEAM_BRAKE_DEBOUNCE);

    private final ArmFeedforward pivotFeedforward = PIVOT_FF;
    private Measure<Angle> pivotSetpoint = PIVOT_INITIAL_POSITION;
    public final SysIdRoutine pivotSysId;

    private final SimpleMotorFeedforward feedFeedforward = FEED_FF;
    private final Measure<Velocity<Angle>> feedSetpoint = null;
    public final SysIdRoutine feedSysId;

    private final SimpleMotorFeedforward flywheelTopFeedforward = FLYWHEEL_TOP_FF;
    private final SimpleMotorFeedforward flywheelBottomFeedforward = FLYWHEEL_BOTTOM_FF;
    private Measure<Velocity<Angle>> flywheelsSetpoint = null;
    public final SysIdRoutine flywheelsSysId;

    @Getter
    private Goal goal = Goal.DEFAULT;

    private Command goal(Goal newGoal) {
        return new FunctionalCommand(
                () -> {
                    goal = newGoal;
                    processGoal();
                },
                () -> {
                },
                (interrupted) -> {
                    goal = Goal.DEFAULT;
                    processGoal();
                },
                // end if the goal changed
                () -> goal != newGoal,
                this
        );
    }

    private static Shooter instance;

    public static Shooter get() {
        if (instance == null)
            synchronized (Shooter.class) {
                instance = new Shooter();
            }

        return instance;
    }

    private Shooter() {
        io = switch (Constants.mode) {
            case REAL -> new ShooterIOSparkMaxBeamBreak(
                    6,
                    7,
                    9,
                    10,
                    8
            );
            case SIM -> new ShooterIOSim(
                    DCMotor.getNEO(1),
                    0.4,
                    0.083,
                    DCMotor.getNEO(1),
                    DCMotor.getNEO(1),
                    DCMotor.getNEO(1)
            );
            case REPLAY -> new ShooterIO();
        };

        io.pivotConfigurePID(PIVOT_PID);
        io.pivotSetPosition(PIVOT_INITIAL_POSITION.in(Radians));

        io.feedConfigurePID(FEED_PID);
        io.flywheelsConfigurePID(FLYWHEEL_TOP_PID, FLYWHEEL_BOTTOM_PID);

        pivotSysId = Util.sysIdRoutine(
                "Shooter/Pivot",
                (voltage) -> io.pivotSetVoltage(voltage.in(Volts)),
                () -> goal = Goal.CHARACTERIZATION,
                () -> goal = Goal.DEFAULT,
                this
        );

        feedSysId = Util.sysIdRoutine(
                "Shooter/Feed",
                (voltage) -> io.feedSetVoltage(voltage.in(Volts)),
                () -> goal = Goal.CHARACTERIZATION,
                () -> goal = Goal.DEFAULT,
                this
        );

        flywheelsSysId = Util.sysIdRoutine(
                "Shooter/Flywheels",
                (voltage) -> io.flywheelsSetVoltage(voltage.in(Volts)),
                () -> goal = Goal.CHARACTERIZATION,
                () -> goal = Goal.DEFAULT,
                this
        );

        var feedCommandRunner = new GoalBasedCommandRunner<>("ShooterFeed", () -> goal);
        new Trigger(() -> goal.feedSetpoint.get().isShoot() && atGoal())
                .onTrue(
                        feedCommandRunner
                                .startEnd(
                                        () -> io.feedSetVoltage(12),
                                        () -> {
                                            io.feedSetVoltage(0);
                                            goal = Goal.DEFAULT;
                                        }
                                )
                                .withTimeout(0.6)
                                .withName("Shooter Feed Shoot")
                );
    }

    private void processGoal() {
        if (goal.goalChange != null)
            goal.goalChange.get().ifPresent((newGoal) -> goal = newGoal);
        Logger.recordOutput("Shooter/Goal", goal);
        pivotSetpoint = goal.pivotSetpoint.get();
        flywheelsSetpoint = goal.flywheelsSetpoint.get();
        var feedSetpoint = goal.feedSetpoint.get();
//        if (feedSetpoint != null)
//            feedSetpoint.ifVelocity((velocity) -> this.feedSetpoint = velocity);
        if (goal == Goal.EJECT) {
            io.feedSetVoltage(12);
        } else if (goal == Goal.HANDOFF_FEED) {
            io.feedSetVoltage(3.5);
        } else if (feedSetpoint != null && !feedSetpoint.isShoot()) {
            io.feedSetVoltage(0);
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Shooter", inputs);

        processGoal();

        Logger.recordOutput("Shooter/Pivot/ClosedLoop", pivotSetpoint != null);
        if (pivotSetpoint != null) {
            Logger.recordOutput("Shooter/Pivot/Setpoint", pivotSetpoint);

            if (DriverStation.isEnabled()) {
                var pivotSetpointRad = pivotSetpoint.in(Radians);
                var ffVolts = pivotFeedforward.calculate(pivotSetpointRad, 0);
                Logger.recordOutput("Shooter/Pivot/FFVolts", ffVolts);
                io.pivotSetSetpoint(pivotSetpointRad, ffVolts);
            }
        }

        Logger.recordOutput("Shooter/Feed/ClosedLoop", feedSetpoint != null);
        if (feedSetpoint != null) {
            Logger.recordOutput("Shooter/Feed/Setpoint", feedSetpoint);

            if (DriverStation.isEnabled()) {
                var feedSetpointRadPerSec = feedSetpoint.in(RadiansPerSecond);
                var ffVolts = feedFeedforward.calculate(feedSetpointRadPerSec, 0);
                Logger.recordOutput("Shooter/Feed/FFVolts", ffVolts);
                io.feedSetSetpoint(feedSetpointRadPerSec, ffVolts);
            }
        }

        Logger.recordOutput("Shooter/Flywheels/ClosedLoop", flywheelsSetpoint != null);
        if (flywheelsSetpoint != null) {
            Logger.recordOutput("Shooter/Flywheels/Setpoint", flywheelsSetpoint);

            if (DriverStation.isEnabled()) {
                var flywheelsSetpointRadPerSec = flywheelsSetpoint.in(RadiansPerSecond);
                var ffTopVolts = flywheelTopFeedforward.calculate(flywheelsSetpointRadPerSec, 0);
                var ffBottomVolts = flywheelBottomFeedforward.calculate(flywheelsSetpointRadPerSec, 0);
                Logger.recordOutput("Shooter/Flywheels/FFTopVolts", ffTopVolts);
                Logger.recordOutput("Shooter/Flywheels/FFBottomVolts", ffBottomVolts);
                io.flywheelsSetSetpoint(flywheelsSetpointRadPerSec, ffTopVolts, ffBottomVolts);
            }
        }

        if (Constants.isSim) {
            Logger.recordOutput("Shooter/Component", new Pose3d(new Translation3d(-0.27, 0, 0.59), new Rotation3d(0, -inputs.pivotPositionRad, 0)));
        }
    }

    @AutoLogOutput
    public boolean hasNoteDebounced() {
        return hasNoteDebouncer.calculate(inputs.hasNote);
    }

    public boolean atGoal() {
        boolean pivotAtSetpoint = pivotSetpoint == null || Math.abs(inputs.pivotPositionRad - pivotSetpoint.in(Radians)) <= PIVOT_SETPOINT_TOLERANCE.in(Radians);
        boolean feedAtSetpoint = feedSetpoint == null || Math.abs(inputs.feedVelocityRadPerSec - feedSetpoint.in(RadiansPerSecond)) <= FEED_SETPOINT_TOLERANCE.in(RadiansPerSecond);
        boolean flywheelsAtSetpoint = flywheelsSetpoint == null ||
                (Math.abs(inputs.flywheelTopVelocityRadPerSec - flywheelsSetpoint.in(RadiansPerSecond)) <= FLYWHEEL_SETPOINT_TOLERANCE.in(RadiansPerSecond) &&
                        Math.abs(inputs.flywheelBottomVelocityRadPerSec - flywheelsSetpoint.in(RadiansPerSecond)) <= FLYWHEEL_SETPOINT_TOLERANCE.in(RadiansPerSecond));
        return pivotAtSetpoint && feedAtSetpoint && flywheelsAtSetpoint;
    }

    public Command handoffWaitForIntake() {
        return goal(Goal.HANDOFF_WAIT_FOR_INTAKE).withName("Shooter Handoff Wait For Intake");
    }

    public Command handoffReady() {
        return goal(Goal.HANDOFF_READY).withName("Shooter Handoff Ready");
    }

    public Command handoffFeed() {
        return goal(Goal.HANDOFF_FEED).withName("Shooter Handoff Feed");
    }

    public Command shootSubwoofer() {
        return goal(Goal.SHOOT_SUBWOOFER).withName("Shooter Shoot Subwoofer");
    }

    public Command shootCalculated() {
        return goal(Goal.SHOOT_CALCULATED).withName("Shooter Shoot Calculated");
    }

    public Command shootConfigurable() {
        return goal(Goal.SHOOT_CONFIGURABLE).withName("Shooter Shoot Configurable");
    }

    public Command amp() {
        return goal(Goal.AMP).withName("Shooter Amp");
    }

    public Command eject() {
        return goal(Goal.EJECT).withName("Shooter Eject");
    }
}
