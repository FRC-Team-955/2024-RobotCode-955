package frc.robot.command.handoff;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooterV1.ShooterV1;

public class IntakeHandoffAuto extends Command {

    private int state = 0;

    public IntakeHandoffAuto() {
        addRequirements(Intake.instance, Shooter.instance);
    }

    @Override
    public void initialize() {
        state = 0;
        Intake.movePositionIntake();
        Intake.setIntakePercent(1);
        Shooter.setPivotPositionLoad();
        Shooter.setIntaking(false);
    }

    @Override
    public void execute() {
        switch (state) {
            case 0: {
                if (Intake.noteCaptured()) {
                    Intake.movePositionHandoff();
                    Intake.setIntakePercent(1);
                    state++;
                }
            }
            break;
            case 1: {
                if (Intake.atSetpoint() && ShooterV1.atPivotSetpoint()) {
                    Intake.setIntakePercentHandoff();
                    Shooter.setIntaking(true);
                    Shooter.setSpinup(true);
                    state++;
                }
            }
            break;
            case 2: {
                if (Shooter.hasNote()) {
                    Intake.movePositionHover();
                    Intake.setIntakePercent(0);
                    Shooter.setPivotPositionSubwoofer();
                    state++;
                }
            }
            break;
            case 3: break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 3;
    }

    @Override
    public void end(boolean interrupted) {
        state = 0;
    }
}
