package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.shooter.Shooter;

public class PrepShootLongAuto extends Command {

    public PrepShootLongAuto() {
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setSpinup(true);
        Shooter.setPivotPosition(Constants.Shooter.Setpoints.autoShootLong);
    }

    @Override
    public boolean isFinished() {
        return Shooter.atPivotSetpoint();
    }
}
