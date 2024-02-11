package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.intake.Intake;

public class IntakeGround extends Command {
    @Override
    public void initialize() {
        Intake.movePosition(Constants.Intake.Setpoints.intake);
        Intake.setIntakePercent(1);
    }

    @Override
    public boolean isFinished() {
        return Intake.noteCaptured();
    }
}
