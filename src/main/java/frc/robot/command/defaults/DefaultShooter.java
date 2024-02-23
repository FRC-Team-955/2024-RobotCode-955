package frc.robot.command.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooterV1.ShooterV1;

public class DefaultShooter extends Command {

    public DefaultShooter() {
        addRequirements(ShooterV1.instance);
    }

    @Override
    public void initialize() {
        ShooterV1.setFlywheelVelocityZero();
        ShooterV1.setPivotPositionTuck();
    }
}
