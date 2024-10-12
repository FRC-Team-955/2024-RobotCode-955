package frc.robot.factories;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class FourPieceWingAutoFactory {
    public static Command get(AutoFactory factory) {
        final var intake = Intake.get();

        final var loop = factory.newLoop("4 Piece Wing");
        final var StoW1 = factory.trajectory("S-W1", loop);
        final var W1toW2 = factory.trajectory("W1-W2", loop);
        final var W2toW3 = factory.trajectory("W2-W3", loop);

        loop.enabled().onTrue(
                AutoInitFactory.get(loop, "S-W1", StoW1::getInitialPose)
                        .andThen(StoW1.cmd().alongWith(intake.intake()))
        );
        StoW1.done().onTrue(Commands.sequence(
                HandoffFactory.get(),
                CalculatedShootFactory.get(() -> 0, () -> 0),
                W1toW2.cmd().alongWith(intake.intake())
        ));
        W1toW2.done().onTrue(Commands.sequence(
                HandoffFactory.get(),
                CalculatedShootFactory.get(() -> 0, () -> 0),
                W2toW3.cmd().alongWith(intake.intake())
        ));
        W2toW3.done().onTrue(Commands.sequence(
                HandoffFactory.get(),
                CalculatedShootFactory.get(() -> 0, () -> 0)
        ));
        return loop.cmd();
    }
}
