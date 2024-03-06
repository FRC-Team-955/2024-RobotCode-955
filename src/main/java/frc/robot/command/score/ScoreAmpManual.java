package frc.robot.command.score;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class ScoreAmpManual extends Command {

    int state = 0;
    private final BooleanSupplier r;
    private boolean aligned = false;
    Timer timer;

    public ScoreAmpManual(BooleanSupplier control) {
        r = control;
        timer = new Timer();
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setPivotPositionAmp();
    }

    @Override
    public void execute() {
        if (!r.getAsBoolean())
            aligned = true;

        switch (state) {
            case 0: {
                if (aligned && Shooter.atPivotSetpoint()) {
                    timer.start();
                    Shooter.setSpinup(true);
                    state++;
                }
            }
            break;
            case 1: {
                if (timer.get() > 1.5) {
                    Shooter.shoot();
                    state++;
                }
            }
            break;
            case 2: break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 2 && !Shooter.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setPivotPositionTuck();
        timer.stop();
        timer.reset();
        state = 0;
        aligned = false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
