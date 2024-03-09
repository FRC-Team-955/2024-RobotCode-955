package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;

public class PrepShootAuto extends Command {

    double a;

    public PrepShootAuto(double angle) {
        a = angle;
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setSpinup(true);
        Shooter.setPivotPosition(a);
    }

    @Override
    public boolean isFinished() {
        return Shooter.atPivotSetpoint() && Shooter.getFlywheelVelocity() > 2500;
    }
}
