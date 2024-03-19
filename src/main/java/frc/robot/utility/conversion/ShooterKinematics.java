package frc.robot.utility.conversion;

import frc.robot.Constants;
import frc.robot.utility.object.InterpolationTableDouble;

public class ShooterKinematics {
    public static final InterpolationTableDouble map;

    static {
        map = new InterpolationTableDouble(Constants.Shooter.distanceInterpMap);
    }

    public static double getAngleForRange(double range) {
        return map.get(range);
    }
}
