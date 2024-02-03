package frc.robot.utility;

import edu.wpi.first.math.geometry.Translation2d;

public class InputUtil {
    public static double deadzone(final double input, final double deadzone) {
        if (input <= deadzone) return 0;
        return input;
    }

    public static Translation2d deadzone(final Translation2d input, final double deadzone) {
        return new Translation2d(deadzone(input.getX(), deadzone), deadzone(input.getY(), deadzone));
    }

    public static double square(final double input) {
        return input * input;
    }

    public static Translation2d square(final Translation2d input) {
        return new Translation2d(square(input.getX()), square(input.getY()));
    }
}
