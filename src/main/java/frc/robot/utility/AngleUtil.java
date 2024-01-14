package frc.robot.utility;

public class AngleUtil {
    /**
     * Transforms a given angle to a 0 to 360 degree range
     * @param angle The angle to be transformed
     * @return The unsigned range of the given angle, from 0 (inclusive) to 360 (exclusive)
     */
    public static double unsignedRangeDegrees(double angle) {
       angle = angle % 360;
       if (angle < 0) angle += 360;
       return angle;
    }

    /**
     * Transforms a given angle to a -180 to 180 degree range
     * @param angle The angle to be transformed
     * @return The signed range of the given angle, from -180 (exclusive) to 180 (inclusive)
     */
    public static double signedRangeDegrees(double angle) {
        angle = angle % 360;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    /**
     * Returns the signed difference between two angles
     * @param reference The angle in degrees for the difference to be relative to
     * @param angle The angle in degrees to check for difference relative to the reference angle
     * @return The signed error in degrees from -180 (exclusive) to 180 (inclusive) between the two angles
     */
    public static double signedRangeDifferenceDegrees(double reference, double angle) {
        return unsignedRangeDegrees(angle - reference);
    }
}
