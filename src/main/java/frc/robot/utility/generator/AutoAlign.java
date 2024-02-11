package frc.robot.utility.generator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensor.pose.Odometry;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class AutoAlign {

    public static Command withNav(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, new PathConstraints(Constants.Swerve.Constraints.maxFreeSpeed,
                Constants.Swerve.Constraints.maxAcceleration, Constants.Swerve.Constraints.maxRotationSpeed,
                Constants.Swerve.Constraints.maxRotationalAcceleration));
    }

    public static Command interpolation(Pose2d pose) {
        ArrayList<PathPoint> points = new ArrayList<PathPoint>();
        points.add(new PathPoint(Odometry.getPose().getTranslation()));
        points.add(new PathPoint(pose.getTranslation()));
        PathPlannerPath path = PathPlannerPath.fromPathPoints(points, new PathConstraints(
                Constants.Swerve.Constraints.maxFreeSpeed, Constants.Swerve.Constraints.maxAcceleration,
                Constants.Swerve.Constraints.maxRotationSpeed, Constants.Swerve.Constraints.maxRotationalAcceleration),
                new GoalEndState(0.0, pose.getRotation()));
        return AutoBuilder.followPath(path);
    }
}
