package frc.robot.command.score;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensor.pose.Odometry;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.utility.conversion.ShooterKinematics;
import frc.robot.utility.information.FieldUtil;

import java.util.function.BooleanSupplier;

public class ScoreSpeakerManual extends Command {

    int state = 0;
    private final BooleanSupplier r;
    private boolean aligned = false;

    public ScoreSpeakerManual(BooleanSupplier control) {
        r = control;
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setSpinup(true);
        Shooter.setPivotPosition(Constants.Shooter.Setpoints.subwoofer);
    }

    @Override
    public void execute() {
//        Pose2d relative = Odometry.getPose().relativeTo(FieldUtil.speakerPose());
//        double range = Math.hypot(relative.getX(), relative.getY());
//        Shooter.setPivotPosition(ShooterKinematics.getAngleForRange(range));

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

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
