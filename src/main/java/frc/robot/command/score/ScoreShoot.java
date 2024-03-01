package frc.robot.command.score;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;

public class ScoreShoot extends Command {

    double a;

    int state = 0;

    public ScoreShoot(double angle) {
        a = angle;
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setPivotPositionSubwoofer();
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
    }
}
