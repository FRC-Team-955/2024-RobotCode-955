package frc.robot.subsystems.shooter;

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
        return -65.957 * Math.pow(0.7816, distance);
    }

    public static double getSpeed(double distance) {
        return 2722 + 557.3 * distance;
    }
}
