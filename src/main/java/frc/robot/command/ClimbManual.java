package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.climber.Climber;

import java.util.function.DoubleSupplier;

public class ClimbManual extends Command {

    DoubleSupplier i;

    public ClimbManual(DoubleSupplier control) {
        i = control;
        addRequirements(Climber.instance);
    }

    @Override
    public void execute() {
        Climber.setVoltage(i.getAsDouble() * 12);
    }
}
