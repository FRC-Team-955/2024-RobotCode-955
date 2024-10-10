package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public final class ShooterRegression {
    /*
    distance to speaker (m) | shooter angle (degrees) | shooter speed (RPM)
    ------------------------|-------------------------|--------------------
    1.337                   | -50                     | 3500
    2.390                   | -34                     | 4000
    3.3                     | -28.5                   | 4250
    4.05                    | -25.5                   | 5000
    */

    public static double getAngle(double distance) {
        // power regression (bit inaccurate)
        //return -65.957 * Math.pow(0.7816, distance);
        // regression inspired by actual kinematics (surprisingly accurate)
        return -Units.radiansToDegrees(Math.atan(1.23351 / (distance - 0.20496))) - 1.90326 * distance;
    }

    public static double getSpeed(double distance) {
        // linear
        // return 2722 + 557.3 * distance;
        // quadratic
        return Math.min(51.0189 * Math.pow(distance, 2) + 276.094 * distance + 3041.66, 6000);
    }
}
