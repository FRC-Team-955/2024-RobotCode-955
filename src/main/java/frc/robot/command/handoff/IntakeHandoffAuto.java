package frc.robot.command.handoff;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooterV1.ShooterV1;

public class IntakeHandoffAuto extends Command {

    private int state = 0;
    private Timer timer;

    public IntakeHandoffAuto() {
        addRequirements(Intake.instance, Shooter.instance);
        timer = new Timer();
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
                    state++;
                }
            }
            break;
            case 1: {
                if (Intake.atSetpoint()) {
                    Shooter.setPivotPositionLoad();
                    Shooter.setSpinup(true);
                    Intake.setIntakePercent(0);
                    state++;
                }
            }
            case 2: {
                if (Intake.atSetpoint() && Shooter.atPivotSetpoint()) {
                    Intake.setIntakePercentHandoff();
                    Shooter.setIntaking(true);
                    timer.start();
                    state++;
                }
            }
            break;
            case 3: {
                if (Shooter.hasNote() || timer.get() > 2) {
                    Shooter.setPivotPositionHover();
                    Intake.movePositionHover();
                    state++;
                }
            }
            break;
            case 4: {
                if (Intake.atSetpoint()) {
                    Shooter.setPivotPositionTuck();
                    Intake.setIntakePercent(0);
                    state++;
                }
            }
            break;
            case 5: break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 5 && Shooter.atPivotSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        state = 0;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
