package frc.robot;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Util {
    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static Pose2d flipIfNeeded(Pose2d pose) {
        if (shouldFlip())
            return GeometryUtil.flipFieldPose(pose);
        else
            return pose;
    }

    public static Translation2d flipIfNeeded(Translation2d pos) {
        if (shouldFlip())
            return GeometryUtil.flipFieldPosition(pos);
        else
            return pos;
    }
}
