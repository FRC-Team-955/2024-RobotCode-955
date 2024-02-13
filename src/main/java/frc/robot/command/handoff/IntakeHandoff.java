package frc.robot.command.handoff;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;

public class IntakeHandoff extends Command {

    private int state  = 0;

    public IntakeHandoff() {

    }

    @Override
    public void initialize() {
        Intake.movePositionIntake();
        Intake.setIntakePercentIntake();
        Shooter.setPivotPositionTuck();
    }

    @Override
    public void execute() {
        switch (state) {
            case 0:
                if (Intake.noteCaptured()) {
                    Intake.movePositionHandoff();
                    state++;
                }
                break;
            default:
                break;
        }
    }
}
