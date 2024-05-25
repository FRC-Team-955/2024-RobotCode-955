package frc.robot.subsystems.shooter;

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
import frc.lib.util.absoluteencoder.AbsoluteEncoder;
import frc.lib.util.absoluteencoder.AbsoluteEncoderIO;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
    private static final ArmFeedforward PIVOT_FF = Constants.mode.isReal() ? new ArmFeedforward(0, 0.59, 0) : new ArmFeedforward(0, 0.01, 0);
    private static final PIDConstants PIVOT_PID = Constants.mode.isReal() ? new PIDConstants(0.1/*, 0.011*/) : new PIDConstants(2.5, 0);
    private static final double PIVOT_GEAR_RATIO = 40;
    private static final Measure<Angle> PIVOT_OFFSET = Radians.of(0.0);
    private static final Measure<Angle> PIVOT_INITIAL_POSITION = Degrees.of(-90);
    private static final Measure<Angle> PIVOT_HOVER = Degrees.of(-10);
    private static final Measure<Angle> PIVOT_HANDOFF = Degrees.of(-45);
    private static final Measure<Angle> PIVOT_SHOOT = Degrees.of(0);

    private static final SimpleMotorFeedforward FEED_FF = Constants.mode.isReal() ? new SimpleMotorFeedforward(0, 0) : new SimpleMotorFeedforward(0, 0.058);
    private static final PIDConstants FEED_PID = Constants.mode.isReal() ? new PIDConstants(0.1, 0.0001) : new PIDConstants(0.1, 0);
    private static final double FEED_GEAR_RATIO = 3;
    private static final Measure<Velocity<Angle>> FEED_SETPOINT_TOLERANCE = RPM.of(10);

    private static final SimpleMotorFeedforward FLYWHEEL_FF = Constants.mode.isReal() ? new SimpleMotorFeedforward(0, 0) : new SimpleMotorFeedforward(0, 0.058);
    private static final PIDConstants FLYWHEEL_PID = Constants.mode.isReal() ? new PIDConstants(0.1, 0.0001) : new PIDConstants(0.1, 0);
    private static final double FLYWHEEL_GEAR_RATIO = 1;
    private static final Measure<Velocity<Angle>> FLYWHEEL_SETPOINT_TOLERANCE = RPM.of(10);

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final ShooterIO io;

    public final Arm pivot;
    private final ArmVisualizer pivotVisualizer = new ArmVisualizer(Color.kOrange, 6, 6, 2.5);
    public final Wheel feed;
    public final Wheel flywheelTop;
    public final Wheel flywheelBottom;

    private static Shooter instance;

    public static Shooter get() {
        return instance;
    }

    public Shooter(ShooterIO io, ArmIO pivotIO, AbsoluteEncoderIO pivotAbsoluteEncoderIO, WheelIO feedIO, WheelIO flywheelTopIO, WheelIO flywheelBottomIO) {
        if (instance != null)
            throw new RuntimeException("Duplicate subsystem created!");
        instance = this;

        this.io = io;

        pivot = new Arm(
                this,
                "Shooter/Pivot",
                pivotIO,
                PIVOT_FF,
                PIVOT_PID,
                PIVOT_GEAR_RATIO,
                PIVOT_INITIAL_POSITION
//                new AbsoluteEncoder(
//                        "Shooter/Pivot/AbsoluteEncoder",
//                        pivotAbsoluteEncoderIO,
//                        PIVOT_GEAR_RATIO,
//                        PIVOT_OFFSET
//                )
        );
        feed = new Wheel(
                this,
                "Shooter/Feed",
                feedIO,
                FEED_FF,
                FEED_PID,
                FEED_GEAR_RATIO,
                FEED_SETPOINT_TOLERANCE
        );
        flywheelTop = new Wheel(
                this,
                "Shooter/FlywheelTop",
                flywheelTopIO,
                FLYWHEEL_FF,
                FLYWHEEL_PID,
                FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_SETPOINT_TOLERANCE
        );
        flywheelBottom = new Wheel(
                this,
                "Shooter/FlywheelBottom",
                flywheelBottomIO,
                FLYWHEEL_FF,
                FLYWHEEL_PID,
                FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_SETPOINT_TOLERANCE
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Shooter", inputs);
        pivot.periodic();
        feed.periodic();
        flywheelTop.periodic();
        flywheelBottom.periodic();

        pivotVisualizer.update(pivot);
        Logger.recordOutput("Shooter/Mechanism", pivotVisualizer.mechanism);
    }

    public Command pivotHover() {
        return pivot.reachSetpointCommand(PIVOT_HOVER);
    }

    public Command pivotHandoff() {
        return pivot.reachSetpointCommand(PIVOT_HANDOFF);
    }

    public Command pivotShoot() {
        return pivot.reachSetpointCommand(PIVOT_SHOOT);
    }

//    public Command feedUntilHasNote() {
//        return feed.setSetpointCommand()
//    }

    public boolean hasNote() {
        return inputs.hasNote;
    }
}
