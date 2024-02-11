package frc.robot.command.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;

public class DefaultShooter extends Command {

    public DefaultShooter() {
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setFlywheelVelocityZero();
        Shooter.setPivotPositionTuck();
    }
}
