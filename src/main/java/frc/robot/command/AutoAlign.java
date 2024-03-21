package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.sensor.pose.Odometry;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.information.FieldUtil;

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

    public static Command correct(Pose2d alignmentPose, double tolerance) {
        PIDController xPid = new PIDController(0.1, 0, 0);
        PIDController yPid = new PIDController(0.1, 0, 0);
        return new FunctionalCommand(() -> {
            Swerve.setTargetHeading(alignmentPose.getRotation().getDegrees());
        }, () -> {
            boolean flip = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
            Swerve.driveSpeeds(new Translation2d(
                xPid.calculate(Odometry.getPose().getX(), alignmentPose.getX()) * (flip ? -1 : 1),
                yPid.calculate(Odometry.getPose().getY(), alignmentPose.getY()) * (flip ? -1 : 1)
            ), 0, true);
        }, (wasInterrupted) -> {
            Swerve.disableHeadingOverride();
        }, () -> {
            return Math.abs(Odometry.getPose().getX() - alignmentPose.getX()) < tolerance &&
                    Math.abs(Odometry.getPose().getY() - alignmentPose.getY()) < tolerance;
        }, Swerve.instance);
    }

    public static Command alignCorrect(Pose2d alignmentPose) {
        return new SequentialCommandGroup(align(alignmentPose), correct(alignmentPose, 0.25));
    }
}
