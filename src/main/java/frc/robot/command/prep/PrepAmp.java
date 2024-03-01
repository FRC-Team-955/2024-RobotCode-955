package frc.robot.command.prep;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;

public class PrepAmp extends Command {

    public PrepAmp() {
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setPivotPositionAmp();
    }
}
