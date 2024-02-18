package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class AmpBasic extends Command {

    int state = 0;
    private final Timer timer;
    private final BooleanSupplier r;
    private boolean aligned = false;

    public AmpBasic(BooleanSupplier control) {
        r = control;
        timer = new Timer();
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
                    Shooter.setFlywheelVelocityMax();
                    Shooter.setNotePosition(1);
                    timer.start();
                    state++;
                }
            }
            break;
            case 1: break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 1 && timer.get() > 0.3;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setPivotPositionTuck();
        Shooter.setFlywheelVelocityZero();
        Shooter.setNotePosition(0);
        state = 0;
        timer.stop();
        timer.reset();
        aligned = false;
    }
}
