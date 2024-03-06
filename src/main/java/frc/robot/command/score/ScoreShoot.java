package frc.robot.command.score;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.utility.conversion.ShooterKinematics;

import java.util.function.BooleanSupplier;

public class ScoreShoot extends Command {

    double d;

    int state = 0;

    public ScoreShoot(double distance) {
        d = distance;
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setPivotPosition(ShooterKinematics.getAngleForRange(d));
    }

    @Override
    public void execute() {
        switch(state) {
            case 0: {
                if (Shooter.atPivotSetpoint()) {
                    state++;
                    Shooter.shoot();
                }
            }
            break;
            case 1: {
                if (!Shooter.hasNote()) {
                    state++;
                    Shooter.setPivotPositionTuck();
                }
            }
            break;
            case 2: break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 2;
    }

    @Override
    public void end(boolean wasInterrupted) {
        state = 0;
        if(wasInterrupted) {
            Shooter.setSpinup(false);
            Shooter.setPivotPositionTuck();
        }
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
