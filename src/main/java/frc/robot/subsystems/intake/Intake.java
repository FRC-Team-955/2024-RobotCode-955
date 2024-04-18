package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.arm.Arm;
import frc.lib.subsystems.arm.ArmIO;
import frc.lib.subsystems.arm.ArmVisualizer;
import frc.lib.subsystems.wheel.Wheel;
import frc.lib.subsystems.wheel.WheelIO;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

public class Intake extends SubsystemBase {
    private static final ArmFeedforward PIVOT_FF = Constants.mode.isReal() ? new ArmFeedforward(0, 0.36, 0) : new ArmFeedforward(0, 0.01, 0);
    private static final PIDConstants PIVOT_PID = Constants.mode.isReal() ? new PIDConstants(0.17, 0.003) : new PIDConstants(2.5, 0);
    private static final SimpleMotorFeedforward FEED_FF = Constants.mode.isReal() ? new SimpleMotorFeedforward(0, 0) : new SimpleMotorFeedforward(0, 0.058);
    private static final PIDConstants FEED_PID = Constants.mode.isReal() ? new PIDConstants(0.17, 0) : new PIDConstants(0.1, 0);

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeIO io;

    public final Arm pivot;
    private final ArmVisualizer pivotVisualizer = new ArmVisualizer(Color.kOrange, 6, 6, 2.5);
    public final Wheel feed;

    private static Intake instance;

    public static Intake get() {
        return instance;
    }

    public Intake(IntakeIO io, ArmIO pivotIO, WheelIO feedIO) {
        if (instance != null)
            throw new RuntimeException("Duplicate subsystem created!");
        instance = this;

        this.io = io;

        pivot = new Arm(
                this,
                "Intake/Pivot",
                pivotIO,
                PIVOT_FF,
                PIVOT_PID,
                9,
                Degrees.of(-175)
        );
        feed = new Wheel(
                this,
                "Intake/Feed",
                feedIO,
                FEED_FF,
                FEED_PID,
                3,
                RPM.of(10)
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
}
