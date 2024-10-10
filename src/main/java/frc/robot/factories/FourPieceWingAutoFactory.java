package frc.robot.factories;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class FourPieceWingAutoFactory {
    public static Command get(AutoFactory factory) {
        final var drive = Drive.get();
        final var shooter = Shooter.get();
        final var intake = Intake.get();

        final var loop = factory.newLoop("4 Piece Wing");
        final var StoW1 = factory.trajectory("S-W1", loop);
        final var W1toW2 = factory.trajectory("W1-W2", loop);
        final var W2toW3 = factory.trajectory("W2-W3", loop);
        loop.enabled().onTrue(
                drive.setPose(() -> StoW1.getInitialPose().get())
                        .andThen(
                                StoW1.cmd().alongWith(intake.intake().withTimeout(5))
                        )
        );
        StoW1.done().onTrue(
                intake.intake().withTimeout(3)
        );
        W1toW2.done().onTrue(
               HandoffFactory.get()
                       .andThen(shooter.eject().withTimeout(2))
                        .andThen(W2toW3.cmd().andThen(Commands.waitSeconds(1)).raceWith(intake.intake()))
        );
        W2toW3.done().onTrue(
                HandoffFactory.get()
        );
        return loop.cmd();
    }
}
