package frc.robot.command.reset;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;

public class SpitOutIntake extends Command {

    int state = 0;
    Timer timer = new Timer();

    public SpitOutIntake() {
        addRequirements(Intake.instance);
    }

    @Override
    public void initialize() {
        Intake.movePositionHover();
    }

    @Override
    public void execute() {
        switch (state) {
            case 0: {
                if (Intake.atSetpoint()) {
                    state++;
                    Intake.setIntakePercent(-1);
                    timer.start();
                }
            }
            break;
            case 1: break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 1 && timer.get() > 0.5;
    }

    @Override
    public void end(boolean wasInterrupted) {
        timer.stop();
        timer.reset();
        state = 0;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
