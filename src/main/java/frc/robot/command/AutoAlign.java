package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.command.prep.PrepAmp;
import frc.robot.command.prep.PrepShoot;
import frc.robot.command.score.ScoreAmp;
import frc.robot.command.score.ScoreShoot;
import frc.robot.sensor.pose.Odometry;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.information.FieldUtil;

import java.util.List;

public class AutoAlign {

    public static Command align(Pose2d alignmentPose) {
        Odometry.setTargetPose(alignmentPose);
        return AutoBuilder.pathfindToPose(alignmentPose, new PathConstraints(Constants.Swerve.Constraints.maxFreeSpeed,
                Constants.Swerve.Constraints.maxAcceleration, Constants.Swerve.Constraints.maxRotationSpeed,
                Constants.Swerve.Constraints.maxRotationalAcceleration), 0)
                .andThen(Commands.runOnce(Odometry::removeTargetPose));

//        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(Odometry.getPose(), alignmentPose);
//        PathPlannerPath path = new PathPlannerPath(bezierPoints, new PathConstraints(
//                Constants.Swerve.Constraints.maxFreeSpeed, Constants.Swerve.Constraints.maxAcceleration,
//                Constants.Swerve.Constraints.maxRotationSpeed, Constants.Swerve.Constraints.maxRotationalAcceleration),
//                new GoalEndState(0.0, alignmentPose.getRotation()));
    }

//    public static Command correct(Pose2d alignmentPose, double tolerance) {
//        return new FunctionalCommand(()->{}, () -> {
//            Swerve.driveSpeeds(new Translation2d(
//
//            ));
//        })
//    }

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
