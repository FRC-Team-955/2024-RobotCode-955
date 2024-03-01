package frc.robot.command.prep;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;

public class PrepShoot extends Command {

    double a;

    public PrepShoot(double angle) {
        a = angle;
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setPivotPosition(a);
        Shooter.setSpinup(true);
    }
}
