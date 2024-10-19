package frc.robot.factories;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class ThreePieceMidlineAutoFactory {
    public static Command get(AutoFactory factory) {
        final var intake = Intake.get();

        final var loop = factory.newLoop("3 Piece Midline");
        final var BStoLP = factory.trajectory("BS-LP", loop);
        final var LPtoM4toTP = factory.trajectory("LP-M4-TP", loop);
        final var TPtoM3toTP = factory.trajectory("TP-M3-TP", loop);

        loop.enabled().onTrue(
                AutoInitFactory.get(loop, "3 Piece Midline", BStoLP::getInitialPose)
                        .andThen(BStoLP.cmd())
        );
        BStoLP.done().onTrue(Commands.sequence(
                CalculatedShootFactory.get(),
                LPtoM4toTP.cmd()
        ));
        LPtoM4toTP.done().onTrue(Commands.sequence(
                HandoffFactory.get(),
                CalculatedShootFactory.get(),
                TPtoM3toTP.cmd().asProxy() // DO NOT REMOVE .asProxy() !!!!
        ));
        TPtoM3toTP.done().onTrue(Commands.sequence(
                HandoffFactory.get(),
                CalculatedShootFactory.get()
        ));

        LPtoM4toTP.atTime("intake").onTrue(intake.intake().withTimeout(1.0));
        TPtoM3toTP.atTime("intake").onTrue(intake.intake().withTimeout(1.0));

//        final var HANDOFF_TIMEOUT = 0.5;
//        LPtoM4toTP.atTime("handoff").onTrue(HandoffFactory.get());
//        TPtoM3toTP.atTime("handoff").onTrue(HandoffFactory.get());

        return loop.cmd();
    }
}
