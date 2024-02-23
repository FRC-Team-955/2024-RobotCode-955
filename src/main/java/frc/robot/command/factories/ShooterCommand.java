package frc.robot.command.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystem.shooterV1.ShooterV1;

public class ShooterCommand {

    public static Command warmup(double angle) {
        return new FunctionalCommand(() -> {
            ShooterV1.setPivotPosition(angle);
            ShooterV1.setFlywheelVelocityMax();
        }, ()->{}, (b)->{}, () -> ShooterV1.atFlywheelSetpoint() && ShooterV1.atPivotSetpoint());
    }
}
