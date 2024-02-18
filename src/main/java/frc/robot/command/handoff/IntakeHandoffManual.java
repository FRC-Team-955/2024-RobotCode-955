package frc.robot.command.handoff;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class IntakeHandoffManual extends Command {

    private int state = 0;
    private final BooleanSupplier r;
    private final Timer timer;

    public IntakeHandoffManual(BooleanSupplier control) {
        r = control;
        timer = new Timer();
        addRequirements(Intake.instance, Shooter.instance);
    }

    @Override
    public void initialize() {
        state = 0;
        Intake.movePositionIntake();
        Intake.setIntakePercent(1);
        Shooter.setPivotPositionLoad();
        Shooter.setFlywheelVelocityZero();
        Shooter.setFlywheelIndexing(false);
    }

    @Override
    public void execute() {
        switch (state) {
            case 0: {
                System.out.println(r.getAsBoolean());
                if (!r.getAsBoolean()) {
                    System.out.println("MISBEHAVE");
                    Intake.movePositionHandoff();
                    Intake.setIntakePercent(1);
                    state++;
                }
            }
            break;
            case 1: {
                if (Intake.atSetpoint() && Shooter.atPivotSetpoint()) {
                    Intake.setIntakePercentHandoff();
                    Shooter.setNotePosition(1);
                    timer.start();
                    state++;
                }
            }
            break;
            case 2: {
                if (timer.get() >= 0.3) {
                    timer.stop();
                    Intake.movePositionHover();
                    Shooter.setPivotPositionTuck();
                    Intake.setIntakePercent(0);
                    Shooter.setNotePosition(0);
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
        System.out.println("IAGSND: " + interrupted);
        state = 0;
        timer.stop();
        timer.reset();
    }
}
