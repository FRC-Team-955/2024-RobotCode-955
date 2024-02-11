package frc.robot.utility.information;

public class MatchUtil {

    private static boolean auto;
    private static boolean teleop;

    /**
     * @return Whether it is before the start of a match
     */
    public static boolean isPrematch() { // TODO
        return !auto;
    }
}
