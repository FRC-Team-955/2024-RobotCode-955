package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    protected static final ArmFeedforward PIVOT_FF = Constants.mode.isReal() ? new ArmFeedforward(0, 0.6, 0) : new ArmFeedforward(0, 0.3, 0);
    protected static final PIDConstants PIVOT_PID = Constants.mode.isReal() ? new PIDConstants(0.12, 0.003) : new PIDConstants(2.5, 0);
    protected static final double PIVOT_GEAR_RATIO = 45;
    private static final Measure<Angle> PIVOT_ENCODER_OFFSET = Radians.of(0.0);
    protected static final Measure<Angle> PIVOT_INITIAL_POSITION = Degrees.of(-141);
    private static final Measure<Angle> PIVOT_CLEAR_OF_SHOOTER = Degrees.of(-130);
    private static final Measure<Angle> PIVOT_HOVER = Degrees.of(-110);
    private static final Measure<Angle> PIVOT_HANDOFF = Degrees.of(-145);
    private static final Measure<Angle> PIVOT_INTAKE = Degrees.of(0);
    private static final Measure<Angle> PIVOT_EJECT = Degrees.of(-70);
    private static final Measure<Angle> PIVOT_SETPOINT_TOLERANCE = Degrees.of(7);

    protected static final SimpleMotorFeedforward FEED_FF = Constants.mode.isReal() ? new SimpleMotorFeedforward(0.5, 1.2) : new SimpleMotorFeedforward(0, 0.058);
    protected static final PIDConstants FEED_PID = Constants.mode.isReal() ? new PIDConstants(0, 0) : new PIDConstants(0.1, 0);
    protected static final double FEED_GEAR_RATIO = 4;
    protected static final Measure<Velocity<Angle>> FEED_SETPOINT_TOLERANCE = RPM.of(10);

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeIO io;

    private final ArmFeedforward pivotFeedforward = PIVOT_FF;
    private Measure<Angle> pivotSetpoint = PIVOT_INITIAL_POSITION;

    private final SimpleMotorFeedforward feedFeedforward = FEED_FF;
    private final Measure<Velocity<Angle>> feedSetpoint = null;

    private static Intake instance;

    public static Intake get() {
        return instance;
    }

    public Intake(IntakeIO io) {
        if (instance != null)
            throw new RuntimeException("Duplicate subsystem created!");
        instance = this;

        this.io = io;

        io.pivotConfigurePID(PIVOT_PID);
        io.pivotSetPosition(PIVOT_INITIAL_POSITION.in(Radians));

        io.feedConfigurePID(FEED_PID);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Intake", inputs);

        Logger.recordOutput("Intake/Pivot/ClosedLoop", pivotSetpoint != null);
        if (pivotSetpoint != null) {
            Logger.recordOutput("Intake/Pivot/Setpoint", pivotSetpoint);

            if (RobotState.isEnabled()) {
                var pivotSetpointRad = pivotSetpoint.in(Radians);
                var ffVolts = pivotFeedforward.calculate(pivotSetpointRad, 0);
                Logger.recordOutput("Intake/Pivot/FFVolts", ffVolts);
                io.pivotSetSetpoint(pivotSetpointRad, ffVolts);
            }
        }

        Logger.recordOutput("Intake/Feed/ClosedLoop", feedSetpoint != null);
        if (feedSetpoint != null) {
            Logger.recordOutput("Intake/Feed/Setpoint", feedSetpoint);

            if (RobotState.isEnabled()) {
                var feedSetpointRadPerSec = feedSetpoint.in(RadiansPerSecond);
                var ffVolts = feedFeedforward.calculate(feedSetpointRadPerSec, 0);
                Logger.recordOutput("Intake/Feed/FFVolts", ffVolts);
                io.feedSetSetpoint(feedSetpointRadPerSec, ffVolts);
            }
        }
    }

    public boolean isClearOfShooter() {
        return inputs.pivotPositionRad >= PIVOT_CLEAR_OF_SHOOTER.in(Radians);
    }

    private boolean pivotAtSetpoint() {
        return Math.abs(inputs.pivotPositionRad - pivotSetpoint.in(Radians)) <= PIVOT_SETPOINT_TOLERANCE.in(Radians);
    }

    private Command pivotSetpoint(Measure<Angle> setpoint) {
        return startEnd(
                () -> pivotSetpoint = setpoint,
                () -> {
                }
        ).until(this::pivotAtSetpoint);
    }

    private Command feedPercent(double percent) {
        return startEnd(
                () -> io.feedSetVoltage(percent * 12.0),
                () -> io.feedSetVoltage(0)
        );
    }

    public Command pivotHover() {
        return pivotSetpoint(PIVOT_HOVER);
    }

    public Command pivotHandoff() {
        return pivotSetpoint(PIVOT_HANDOFF);
    }

    private Command pivotIntake() {
        return pivotSetpoint(PIVOT_INTAKE);
    }

    private Command pivotEject() {
        return pivotSetpoint(PIVOT_EJECT);
    }

    private Command feedUntilHasNote() {
        return feedPercent(0.5).until(() -> inputs.hasNote);
    }

    public Command intake() {
        return pivotIntake().andThen(feedUntilHasNote());
    }

    public Command feedHandoff() {
        return feedPercent(-0.15);
    }

    public Command eject() {
        return pivotEject().andThen(feedPercent(-0.1));
    }
}
