package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.arm.Arm;
import frc.lib.subsystems.arm.ArmIO;
import frc.lib.subsystems.arm.ArmVisualizer;
import frc.lib.subsystems.wheel.Wheel;
import frc.lib.subsystems.wheel.WheelIO;
import frc.lib.util.absoluteencoder.AbsoluteEncoderIO;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    private static final ArmFeedforward PIVOT_FF = Constants.mode.isReal() ? new ArmFeedforward(0, 0.6, 0) : new ArmFeedforward(0, 0.3, 0);
    private static final PIDConstants PIVOT_PID = Constants.mode.isReal() ? new PIDConstants(0.12, 0.003) : new PIDConstants(2.5, 0);
    private static final double PIVOT_GEAR_RATIO = 45;
    private static final Measure<Angle> PIVOT_ENCODER_OFFSET = Radians.of(0.0);
    private static final Measure<Angle> PIVOT_INITIAL_POSITION = Degrees.of(-141);
    private static final Measure<Angle> PIVOT_CLEAR_OF_SHOOTER = Degrees.of(-130);
    private static final Measure<Angle> PIVOT_HOVER = Degrees.of(-110);
    private static final Measure<Angle> PIVOT_HANDOFF = Degrees.of(-145);
    private static final Measure<Angle> PIVOT_INTAKE = Degrees.of(0);
    private static final Measure<Angle> PIVOT_EJECT = Degrees.of(-70);

    private static final SimpleMotorFeedforward FEED_FF = Constants.mode.isReal() ? new SimpleMotorFeedforward(0.5, 1.2) : new SimpleMotorFeedforward(0, 0.058);
    private static final PIDConstants FEED_PID = Constants.mode.isReal() ? new PIDConstants(0, 0) : new PIDConstants(0.1, 0);
    private static final double FEED_GEAR_RATIO = 4;
    private static final Measure<Velocity<Angle>> FEED_SETPOINT_TOLERANCE = RPM.of(10);


    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeIO io;

    private final Arm pivot;
    private final ArmVisualizer pivotVisualizer = new ArmVisualizer(Color.kOrange, 6, 6, 2.5);
    private final Wheel feed;

    private static Intake instance;

    public static Intake get() {
        return instance;
    }

    public Intake(IntakeIO io, ArmIO pivotIO, AbsoluteEncoderIO pivotAbsoluteEncoderIO, WheelIO feedIO) {
        if (instance != null)
            throw new RuntimeException("Duplicate subsystem created!");
        instance = this;

        this.io = io;

        pivot = new Arm(
                "Intake/Pivot",
                pivotIO,
                PIVOT_FF,
                PIVOT_PID,
                PIVOT_GEAR_RATIO,
                PIVOT_INITIAL_POSITION
//                pivotAbsoluteEncoderIO,
//                PIVOT_OFFSET
        );
        feed = new Wheel(
                "Intake/Feed",
                feedIO,
                FEED_FF,
                FEED_PID,
                FEED_GEAR_RATIO,
                FEED_SETPOINT_TOLERANCE
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Intake", inputs);
        pivot.periodic();
        feed.periodic();

        pivotVisualizer.update(pivot);
        Logger.recordOutput("Intake/Mechanism", pivotVisualizer.mechanism);
    }

    public boolean isClearOfShooter() {
        return pivot.getPosition().gte(PIVOT_CLEAR_OF_SHOOTER);
    }

    private Command pivotSetpoint(Measure<Angle> setpoint) {
        return startEnd(
                () -> pivot.setSetpoint(setpoint),
                () -> {
                }
        ).until(pivot::atSetpoint);
    }

    private Command feedPercent(double percent) {
        return startEnd(
                () -> feed.setPercent(percent),
                feed::stop
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
