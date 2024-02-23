package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;

public class HandoffCheck extends Command {
    @Override
    public boolean isFinished() {
        return Shooter.hasNote();
    }
}
