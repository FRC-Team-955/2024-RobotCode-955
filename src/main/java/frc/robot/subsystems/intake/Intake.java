package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    public enum Goal {
        HOVER(() -> Degrees.of(-110), RPM::zero),
        INTAKE(() -> Degrees.of(0), RPM::zero),
        EJECT(() -> Degrees.of(-70), RPM::zero),

        HANDOFF_READY(() -> Degrees.of(-145), RPM::zero),
        HANDOFF_FEED(() -> Degrees.of(-145), () -> RPM.of(-100));

        public static final Goal DEFAULT = Goal.HOVER;

        public final Supplier<Measure<Angle>> pivotSetpoint;
        public final Supplier<Measure<Velocity<Angle>>> feedSetpoint;

        Goal(Supplier<Measure<Angle>> pivotSetpoint, Supplier<Measure<Velocity<Angle>>> feedSetpoint) {
            this.pivotSetpoint = pivotSetpoint;
            this.feedSetpoint = feedSetpoint;
        }
    }

    protected static final ArmFeedforward PIVOT_FF = Constants.isReal ? new ArmFeedforward(0, 0.6, 0) : new ArmFeedforward(0, 0.28, 0);
    protected static final PIDConstants PIVOT_PID = Constants.isReal ? new PIDConstants(0.12, 0.003) : new PIDConstants(8, 0);
    protected static final double PIVOT_GEAR_RATIO = 45;
    protected static final Measure<Angle> PIVOT_INITIAL_POSITION = Degrees.of(-141);
    private static final Measure<Angle> PIVOT_SETPOINT_TOLERANCE = Degrees.of(5);

    protected static final SimpleMotorFeedforward FEED_FF = Constants.isReal ? new SimpleMotorFeedforward(0.5, 1.2) : new SimpleMotorFeedforward(0, 0.058);
    protected static final PIDConstants FEED_PID = Constants.isReal ? new PIDConstants(0, 0) : new PIDConstants(0.1, 0);
    protected static final double FEED_GEAR_RATIO = 4;
    protected static final Measure<Velocity<Angle>> FEED_SETPOINT_TOLERANCE = RPM.of(10);

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeIO io;

    private final ArmFeedforward pivotFeedforward = PIVOT_FF;
    private Measure<Angle> pivotSetpoint = null;

    private final SimpleMotorFeedforward feedFeedforward = FEED_FF;
    private Measure<Velocity<Angle>> feedSetpoint = null;

    @Getter
    private Goal goal = Goal.DEFAULT;

    private Command goal(Goal newGoal) {
        return startEnd(
                () -> {
                    goal = newGoal;
                    processGoal();
                },
                () -> {
                    goal = Goal.DEFAULT;
                    processGoal();
                }
        );
    }

    private static Intake instance;

    public static Intake get() {
        if (instance == null)
            synchronized (Intake.class) {
                instance = new Intake();
            }

        return instance;
    }

    private Intake() {
        io = switch (Constants.mode) {
            case REAL -> new IntakeIOSparkMax(
                    3,
                    16
            );
            case SIM -> new IntakeIOSim(
                    DCMotor.getNEO(1),
                    0.3,
                    0.045,
                    DCMotor.getNEO(1)
            );
            case REPLAY -> new IntakeIO();
        };

        io.pivotConfigurePID(PIVOT_PID);
        io.pivotSetPosition(PIVOT_INITIAL_POSITION.in(Radians));

        io.feedConfigurePID(FEED_PID);
    }

    private void processGoal() {
        Logger.recordOutput("Intake/Goal", goal);
        pivotSetpoint = goal.pivotSetpoint.get();
        feedSetpoint = goal.feedSetpoint.get();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Intake", inputs);

        processGoal();

        Logger.recordOutput("Intake/Pivot/ClosedLoop", pivotSetpoint != null);
        if (pivotSetpoint != null) {
            Logger.recordOutput("Intake/Pivot/Setpoint", pivotSetpoint);

            if (DriverStation.isEnabled()) {
                var pivotSetpointRad = pivotSetpoint.in(Radians);
                var ffVolts = pivotFeedforward.calculate(pivotSetpointRad, 0);
                Logger.recordOutput("Intake/Pivot/FFVolts", ffVolts);
                io.pivotSetSetpoint(pivotSetpointRad, ffVolts);
            }
        }

        Logger.recordOutput("Intake/Feed/ClosedLoop", feedSetpoint != null);
        if (feedSetpoint != null) {
            Logger.recordOutput("Intake/Feed/Setpoint", feedSetpoint);

            if (DriverStation.isEnabled()) {
                var feedSetpointRadPerSec = feedSetpoint.in(RadiansPerSecond);
                var ffVolts = feedFeedforward.calculate(feedSetpointRadPerSec, 0);
                Logger.recordOutput("Intake/Feed/FFVolts", ffVolts);
                io.feedSetSetpoint(feedSetpointRadPerSec, ffVolts);
            }
        }

        if (Constants.isSim) {
            Logger.recordOutput("Intake/Component", new Pose3d(new Translation3d(0.27, 0, 0.22), new Rotation3d(0, inputs.pivotPositionRad + Units.degreesToRadians(-10), 0)));
        }
    }

    public boolean pivotAtSetpoint() {
        return Math.abs(inputs.pivotPositionRad - pivotSetpoint.in(Radians)) <= PIVOT_SETPOINT_TOLERANCE.in(Radians);
    }

    public boolean feedAtSetpoint() {
        return Math.abs(inputs.feedVelocityRadPerSec - feedSetpoint.in(RadiansPerSecond)) <= FEED_SETPOINT_TOLERANCE.in(RadiansPerSecond);
    }

    public Command hover() {
        return goal(Goal.HOVER).withName("Intake Handoff");
    }

    public Command handoffReady() {
        return goal(Goal.HANDOFF_READY).withName("Intake Handoff Ready");
    }

    public Command handoffFeed() {
        return goal(Goal.HANDOFF_FEED).withName("Intake Handoff Feed");
    }

    public Command intake() {
        return goal(Goal.INTAKE).withName("Intake Intake");//.until(() -> inputs.hasNote);
    }

    public Command eject() {
        return goal(Goal.EJECT).withName("Intake Eject");
    }
}
