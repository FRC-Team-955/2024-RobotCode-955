package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.command.prep.PrepAmp;
import frc.robot.command.prep.PrepShoot;
import frc.robot.command.score.ScoreAmp;
import frc.robot.command.score.ScoreShoot;
import frc.robot.utility.information.FieldUtil;

public class AutoAlign {

    public static Command align(Pose2d alignmentPose) {
        return AutoBuilder.pathfindToPose(alignmentPose, new PathConstraints(Constants.Swerve.Constraints.maxFreeSpeed,
                Constants.Swerve.Constraints.maxAcceleration, Constants.Swerve.Constraints.maxRotationSpeed,
                Constants.Swerve.Constraints.maxRotationalAcceleration), 0);
    }

    public static Command amp() {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(align(FieldUtil.ampScorePose()), new PrepAmp()),
                new ScoreAmp()
        );
    }

    public static Command subwoofer() {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(align(FieldUtil.subwooferScorePose()),
                        new PrepShoot(Constants.Shooter.Setpoints.subwoofer)),
                new ScoreShoot(Constants.Shooter.Setpoints.subwoofer)
        );
    }
}
