package frc.robot.command.score;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class ScoreAmpManual extends Command {

    int state = 0;
    private final BooleanSupplier r;
    private boolean aligned = false;

    public ScoreAmpManual(BooleanSupplier control) {
        r = control;
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
                    Shooter.shoot();
                    state++;
                }
            }
            break;
            case 1: break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 1 && !Shooter.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setPivotPositionTuck();
        state = 0;
        aligned = false;
    }
}
