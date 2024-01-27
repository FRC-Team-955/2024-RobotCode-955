package frc.robot.utility;

public class MatchUtil {

    private static boolean auto;
    private static boolean teleop;

    public static boolean isPrematch() {
        return !auto;
    }
}
