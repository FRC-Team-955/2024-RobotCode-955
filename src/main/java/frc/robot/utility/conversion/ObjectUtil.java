package frc.robot.utility.conversion;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ObjectUtil {
    public static ChassisSpeeds limitChassisSpeeds(ChassisSpeeds speeds, double vMax, double limit) {
        double v = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double percent = v / vMax;
        if (percent < limit)
            return speeds;
        return speeds.times(limit / percent);
    }
}
