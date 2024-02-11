package frc.robot.command.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.climber.Climber;

public class DefaultClimber extends Command {

    public DefaultClimber() {
        addRequirements(Climber.instance);
    }

    @Override
    public void initialize() {
        Climber.setTargetRetracted();
    }
}
