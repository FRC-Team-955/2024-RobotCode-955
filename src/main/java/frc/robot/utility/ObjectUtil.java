package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ObjectUtil {
    public static Translation2d toTranslation(ChassisSpeeds speeds) {
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }
    public static Rotation2d toRotation(ChassisSpeeds speeds) {
        return new Rotation2d(speeds.omegaRadiansPerSecond);
    }
    public static Transform2d toTransform(ChassisSpeeds speeds) {
        return new Transform2d(toTranslation(speeds), toRotation(speeds));
    }
}
