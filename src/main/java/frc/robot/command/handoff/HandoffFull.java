package frc.robot.command.handoff;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;

public class HandoffFull extends Command {

    int state = 0;

    Timer timer;

    public HandoffFull() {
        timer = new Timer();
        addRequirements(Intake.instance, Shooter.instance);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return state == 0 ? InterruptionBehavior.kCancelSelf : InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public void initialize() {
        Intake.movePositionHandoff();
        Shooter.setPivotPositionLoad();
        Shooter.setFlywheelVelocityZero();
        Shooter.setFlywheelIndexing(false);
    }

    @Override
    public void execute() {
        switch (state) {
            case 0:
                if (Intake.atSetpoint() && Shooter.atPivotSetpoint()) {
                    Intake.setIntakePercentHandoff();
                    Shooter.setNotePosition(1);
                    timer.start();
                    state++;
                }
                break;
            case 1:
                if (timer.get() >= 0.3) {
                    timer.stop();
                    Intake.movePositionHover();
                    Shooter.setPivotPositionTuck();
                    Intake.setIntakePercent(0);
                    Shooter.setNotePosition(0);
                    state++;
                }
                break;
            case 2:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 2;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setFlywheelIndexing(false);
        state = 0;
        timer.stop();
        timer.reset();
//        if (!interrupted)
//            Shooter.setNotePositionSafe(Constants.Shooter.ContactRanges.flywheelStart);
    }
}
