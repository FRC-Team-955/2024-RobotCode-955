package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;

public class IntakeGround extends Command {

    boolean r = false;
    Timer timer = new Timer();

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
        if (!r && Intake.hasNote()) {
            System.out.println("w4aebfh egxbfouhjlk;g ");
            Intake.movePositionHandoff();
            r = true;
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
//        return false;
        return timer.get() > 1 && Intake.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        r = false;
        Intake.movePositionHandoff();
        Intake.setIntakePercent(0);
    }
}
