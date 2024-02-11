package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.shooter.Shooter;

public class ShootSubwoofer extends Command {
    @Override
    public void initialize() {
        Shooter.setFlywheelIndexing(false);
        Shooter.setFlywheelVelocity(5);
    }

    @Override
    public void execute() {
        if (Shooter.atFlywheelSetpoint())
            Shooter.setNotePosition(Constants.Shooter.ContactRanges.exit);
    }

    @Override
    public boolean isFinished() {
        return !Shooter.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setFlywheelVelocity(0);
    }
}
