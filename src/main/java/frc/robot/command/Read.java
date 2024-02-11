package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Read extends Command {
    public Read(Subsystem subsystem) {
        addRequirements(subsystem);
    }
}
