package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

public class AutoAlign {

    public static Command align(Pose2d alignmentPose) {
        return AutoBuilder.pathfindToPose(alignmentPose, new PathConstraints(Constants.Swerve.Constraints.maxFreeSpeed,
                Constants.Swerve.Constraints.maxAcceleration, Constants.Swerve.Constraints.maxRotationSpeed,
                Constants.Swerve.Constraints.maxRotationalAcceleration), 0);
    }

    public static Command amp() {
        return new SequentialCommandGroup(
//                new ParallelDeadlineGroup();
        );
    }
}
