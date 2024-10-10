package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class HandoffFactory {
    public static Command get() {
        final var shooter = Shooter.get();
        final var intake = Intake.get();

        return Commands.sequence(
                shooter.pivotWaitForIntake(),
                intake.pivotHandoff(),
                shooter.pivotHandoff(),
                Commands.race(
                        intake.feedHandoff(),
                        shooter.feedHandoff()
                ),
                shooter.pivotWaitForIntake(),
                intake.pivotHover()
        );
    }
}
