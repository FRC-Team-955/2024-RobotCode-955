package frc.robot.utility.information;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldUtil {

    public static Pose2d ampScorePose() {
        return new Pose2d();
    }

    public static Pose2d subwooferScorePose() {
        return new Pose2d();
    }

    public static Pose2d speakerPose() {
        return shouldFlip() ? new Pose2d(new Translation2d(16.55, 5.55), new Rotation2d(0)) :
                new Pose2d(new Translation2d(0, 5.5), new Rotation2d(0));
    }

    /**
     * Returns true if red, false if blue. Blue is the side with the coordinate origin so basically just flip the pose if this returns true
     */
    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
}
