package frc.robot.command.handoff;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class IntakeHandoffTest extends Command {

    private int state = 0;
    private BooleanSupplier r;

    public IntakeHandoffTest() {
        addRequirements(Intake.instance, Shooter.instance);
    }

    @Override
    public void initialize() {
        state = 0;
        Intake.movePositionIntake();
        Intake.setIntakePercent(1);
        Shooter.setPivotPositionHover();
    }

    @Override
    public void execute() {
        switch (state) {
            case 0: {
                if (Intake.hasNote() && Intake.atSetpoint()) {
                    Intake.movePositionHandoff();
                    Intake.setIntakePercent(1);
                    state++;
                }
            }
            break;
            case 1: {
                if (Intake.atSetpoint()) {
                    Shooter.setPivotPositionLoad();
                    state++;
                }
            }
            break;
            case 2: {
                if (Intake.atSetpoint() && Shooter.atPivotSetpoint()) {
                    Intake.setIntakePercentHandoff();
                    Shooter.setIntaking(true);
                    state++;
                }
            }
            break;
            case 3: {
                if (Shooter.hasNote()) {
                    Shooter.setPivotPositionHover();
                    Intake.setIntakePercent(0);
                    state++;
                }
            }
            break;
            case 4: {
                if (Shooter.atPivotSetpoint()) {
                    Intake.movePositionHover();
                    state++;
                }
            }
            break;
            case 5: {
                if (Intake.atSetpoint()) {
                    Shooter.setPivotPositionTuck();
                    state++;
                }
            }
            break;
            case 6: break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 6;
    }

    @Override
    public void end(boolean interrupted) {
        state = 0;
    }
}
