package frc.robot.command.handoff;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooterV1.ShooterV1;

import java.util.function.BooleanSupplier;

public class IntakeHandoffManual extends Command {

    private int state = 0;
    private final BooleanSupplier r;
    private final Timer timer;

    public IntakeHandoffManual(BooleanSupplier control) {
        r = control;
        timer = new Timer();
        addRequirements(Intake.instance, ShooterV1.instance);
    }

    @Override
    public void initialize() {
        state = 0;
        Intake.movePositionIntake();
        Intake.setIntakePercent(1);
        ShooterV1.setPivotPositionLoad();
        ShooterV1.setFlywheelVelocityZero();
        ShooterV1.setFlywheelIndexing(false);
    }

    @Override
    public void execute() {
        switch (state) {
            case 0: {
                if (!r.getAsBoolean()) {
                    Intake.movePositionHandoff();
                    Intake.setIntakePercent(1);
                    state++;
                }
            }
            break;
            case 1: {
                if (Intake.atSetpoint() && ShooterV1.atPivotSetpoint()) {
                    Intake.setIntakePercentHandoff();
                    ShooterV1.setFeedPercent(1);
                    timer.start();
                    state++;
                }
            }
            break;
            case 2: {
                if (timer.get() >= 0.3) {
                    timer.stop();
                    Intake.movePositionHover();
                    ShooterV1.setPivotPositionTuck();
                    Intake.setIntakePercent(0);
                    ShooterV1.setFeedPercent(0);
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
