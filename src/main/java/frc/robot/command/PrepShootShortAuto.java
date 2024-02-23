package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.shooter.Shooter;

public class PrepShootShortAuto extends Command {

    public PrepShootShortAuto() {
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setSpinup(true);
        Shooter.setPivotPosition(Constants.Shooter.Setpoints.autoShootShort);
    }

    @Override
    public boolean isFinished() {
        return Shooter.atPivotSetpoint();
    }
}
