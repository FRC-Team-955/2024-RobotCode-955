package frc.robot.command;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.swerve.Swerve;

public class DriveCommand extends Command {

    double dx;
    double dy;
    double dr;
    double dtime;

    private final Timer timer;

    public DriveCommand(double x, double y, double r, double time) {
        addRequirements(Swerve.instance);
        dx = x;
        dy = y;
        dr = r;
        dtime = time;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        Swerve.driveSpeeds(new Translation2d(dx, dy), dr, false);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= dtime;
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.driveSpeeds(new Translation2d(0, 0), 0, false);
    }
}
