package frc.robot.command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.shooter.Shooter;

public class ShootAuto extends InstantCommand {
    public ShootAuto() {
        addRequirements(Shooter.instance);
    }

    @Override
    public void execute() {
        Shooter.shoot();
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setSpinup(false);
    }
}
