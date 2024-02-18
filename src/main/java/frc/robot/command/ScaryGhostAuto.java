package frc.robot.command;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.swerve.Swerve;

public class ScaryGhostAuto extends Command {

    int state = 0;

    Timer timer = new Timer();

    public ScaryGhostAuto() {
        addRequirements(Swerve.instance, Shooter.instance, Climber.instance);
    }

    @Override
    public void initialize() {
        Swerve.drivePercents(new Translation2d(0, 0), 0, false);
        Shooter.setPivotPositionLoad();
        Intake.movePositionHandoff();
    }

    @Override
    public void execute() {
        switch (state) {
            case 0: {
                if (Shooter.atPivotSetpoint()) {
                    state++;
                    Shooter.setNotePosition(1);
                    Shooter.setFlywheelVelocityMax();
                    Intake.setIntakePercentHandoff();
                    timer.start();
                }
            }
            case 1: {
                if (timer.get() >= 5) {
                    state++;
                    Shooter.setNotePosition(0);
                    Shooter.setFlywheelVelocityZero();
                    Intake.setIntakePercent(0);
                    Swerve.drivePercents(new Translation2d(-0.3, 0), 0, false);
                }
            }
            case 2: {
                if (timer.get() >= 8) {
                    state++;
                    Swerve.drivePercents(new Translation2d(0, 0), 0, false);
                }
            }
            default: break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 3;
    }

    @Override
    public void end(boolean i) {

    }
}
