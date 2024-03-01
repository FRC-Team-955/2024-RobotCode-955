package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;

public class Spit extends Command {

    Timer timer;

    @Override
    public void initialize() {
        Intake.setIntakePercent(-1);
        timer = new Timer();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 2;
    }

    @Override
    public void end(boolean wasInterrupted) {
        Intake.setIntakePercent(0);
        timer.stop();
        timer.reset();
    }
}
