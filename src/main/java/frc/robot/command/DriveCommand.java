package frc.robot.command;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.swerve.Swerve;

public class DriveCommand extends Command {

    Timer timer;

    double dx;
    double dy;
    double dr;
    double dt;

    public DriveCommand(double x, double y, double r, double t) {
        timer = new Timer();
        dx = x;
        dy = y;
        dr = r;
        dt = t;
    }

    @Override
    public void initialize() {
        timer.start();
        Swerve.driveSpeeds(new Translation2d(dx, dy), dr, false);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= dt;
    }

    @Override
    public void end(boolean wasInterrupted) {
        Swerve.driveSpeeds(new Translation2d(0, 0), 0, false);
    }
}
