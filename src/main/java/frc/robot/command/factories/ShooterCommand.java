package frc.robot.command.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystem.shooter.Shooter;

public class ShooterCommand {

    public static Command warmup(double angle) {
        return new FunctionalCommand(() -> {
            Shooter.setPivotPosition(angle);
            Shooter.setFlywheelVelocityMax();
        }, ()->{}, (b)->{}, () -> Shooter.atFlywheelSetpoint() && Shooter.atPivotSetpoint());
    }
}
