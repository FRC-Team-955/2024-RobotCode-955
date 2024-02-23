package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.intake.Intake;

public class IntakeGround extends Command {

    boolean r = false;

    public IntakeGround() {
        addRequirements(Intake.instance);
    }

    @Override
    public void initialize() {
        Intake.movePositionIntake();
        Intake.setIntakePercent(1);
    }

    @Override
    public void execute() {
        if (!r && Intake.noteCaptured()) {
            Intake.movePositionHandoff();
            r = true;
        }
    }

    @Override
    public boolean isFinished() {
//        return false;
        return r && Intake.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
//        System.out.println(interrupted);
        Intake.movePositionHandoff();
        Intake.setIntakePercent(0);
    }
}
