package frc.robot.utility.information;

import edu.wpi.first.math.geometry.Translation2d;

public class InputUtil {
    /**
     * Deadzones the given input by the given amount,
     * returning zero if the absolute value of the input is less than the deadzone amount
     * @param input The input to be deadzoned
     * @param deadzone The amount to deadzone by
     * @return The deadzoned input
     */
    public static double deadzone(final double input, final double deadzone) {
        if (input <= deadzone) return 0;
        return input;
    }

    /**
     * Deadzones each axis of the given {@link Translation2d} by the given amount,
     * setting the value of the axis zero if the absolute value of the xis is less than the deadzone amount
     * @param input The {@link Translation2d} to be deadzoned
     * @param deadzone The amount to deadzone by
     * @return The deadzoned {@link Translation2d}
     */
    public static Translation2d deadzone(final Translation2d input, final double deadzone) {
        return new Translation2d(deadzone(input.getX(), deadzone), deadzone(input.getY(), deadzone));
    }

    /**
     * Squares the given inputs
     * @param input The input to be squared
     * @return The squared input
     */
    public static double square(final double input) {
        return input * input;
    }

    /**
     * Squares each axis of the given {@link Translation2d}
     * @param input The {@link Translation2d} to be squared
     * @return The squared {@link Translation2d}
     */
    public static Translation2d square(final Translation2d input) {
        return new Translation2d(square(input.getX()), square(input.getY()));
    }
}
