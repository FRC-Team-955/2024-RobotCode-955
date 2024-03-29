package frc.robot.command.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;

public class DefaultIntake extends Command {

    public DefaultIntake() {
        addRequirements(Intake.instance);
    }

    @Override
    public void initialize() {
        if (Intake.hasNote())
            Intake.setIntakePercentHold();
        else
            Intake.setIntakePercent(0);

        Intake.movePositionHover();
    }
}
