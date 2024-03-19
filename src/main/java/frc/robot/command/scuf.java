package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;

public class scuf extends Command {

    private int state = 0;
    Timer timer = new Timer();

    public scuf() {
        addRequirements(Intake.instance);
    }

    @Override
    public void initialize() {
        state = 0;
        Intake.movePositionIntake();
        Intake.setIntakePercent(1);
    }

    @Override
    public void execute() {
        switch (state) {
            case 0: {
                if (Intake.hasNote() && Intake.atDeploySetpoint()) {
                    Intake.movePositionHover();
                    Intake.setIntakePercent(0);
                    state++;
                }
            }
            break;
            case 1: {
                if (Intake.atDeploySetpoint()) {
                    Intake.setIntakePercentHandoff();
                    state++;
                }
            }
            case 2: {
                if (!Intake.hasNote()) {
                    timer.start();
                    state++;
                }
            }
            break;
            case 3: {
                if (timer.get() > 1) {
                    Intake.setIntakePercent(0);
                    state++;
                }
            }
            break;
            case 4: break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 4;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        state = 0;
    }
}
