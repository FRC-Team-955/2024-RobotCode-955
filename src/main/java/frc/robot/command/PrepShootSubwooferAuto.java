package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;

public class PrepShootSubwooferAuto extends Command {

    public PrepShootSubwooferAuto() {
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setSpinup(true);
        Shooter.setPivotPositionSubwoofer();
    }

    @Override
    public boolean isFinished() {
        return Shooter.atPivotSetpoint();
    }
}
