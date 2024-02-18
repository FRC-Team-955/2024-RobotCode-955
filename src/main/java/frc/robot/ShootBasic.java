package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class ShootBasic extends Command {

    int state = 0;
    private final BooleanSupplier r;
    private final Timer timer;
    private boolean aligned = false;

    public ShootBasic(BooleanSupplier control) {
        r = control;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        Shooter.setPivotPositionSubwoofer();
        Shooter.setFlywheelVelocityMax();
        timer.start();
    }

    @Override
    public void execute() {

        if (!r.getAsBoolean())
            aligned = true;

        switch (state) {
            case 0: {
                if (aligned && Shooter.atPivotSetpoint() && timer.get() > 0.5) {
                    Shooter.setNotePosition(1);
                    timer.reset();
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
        return state == 1 && timer.get() > 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setPivotPositionTuck();
        Shooter.setFlywheelVelocityZero();
        Shooter.setNotePosition(0);
        state = 0;
        aligned = false;
        timer.stop();
        timer.reset();
    }
}
