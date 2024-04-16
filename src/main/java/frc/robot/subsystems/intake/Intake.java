package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.arm.Arm;
import frc.lib.subsystems.arm.ArmIO;

import static edu.wpi.first.units.Units.Degrees;

public class Intake extends SubsystemBase {
    private final Arm pivot;

    private static Intake instance;

    public static Intake get() {
        return instance;
    }

    public Intake(ArmIO pivotIO) {
        if (instance != null)
            throw new RuntimeException("Duplicate subsystem created!");
        instance = this;

        pivot = new Arm(
                this,
                "Intake/Pivot",
                pivotIO,
                new ArmFeedforward(0, 0.36, 0),
                new PIDConstants(0.17, 0.003),
                45,
                Degrees.of(175)
        );
    }

    @Override
    public void periodic() {
        pivot.periodic();
    }
}
