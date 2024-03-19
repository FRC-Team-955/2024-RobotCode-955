package frc.robot.command.reset;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;

public class AbortHandoff extends Command {

    int state = 0;

    public AbortHandoff() {
        addRequirements(Shooter.instance, Intake.instance);
    }

    @Override
    public void initialize() {
        Shooter.abortHandoff();
        Shooter.setPivotPositionHover();
        Intake.movePositionHover();
    }

    @Override
    public void execute() {
        switch (state) {
            case 0: {
                if (Intake.atDeploySetpoint()) {
                    state++;
                    Shooter.setPivotPositionTuck();
                }
            }
            case 1: break;
        }
    }

    @Override
    public boolean isFinished() {
        return Intake.atDeploySetpoint() && Shooter.atPivotSetpoint();
    }

    @Override
    public void end(boolean wasInterrupted) {
        state = 0;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
