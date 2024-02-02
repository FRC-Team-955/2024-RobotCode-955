package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    public static final double frameX = 0.6858;
    public static final double frameY = 0.6858;

    public static final class Swerve {

        // 2 Inches
        public static final double wheelRadius = 0.0508;
        // 2.625 inches
        public static final double wheelInset = 0.066675;

        public static final Translation2d[] modulePositions = new Translation2d[] {
                new Translation2d( ((frameX / 2) - wheelInset),  ((frameY / 2) - wheelInset)),
                new Translation2d(-((frameX / 2) - wheelInset),  ((frameY / 2) - wheelInset)),
                new Translation2d(-((frameX / 2) - wheelInset), -((frameY / 2) - wheelInset)),
                new Translation2d( ((frameX / 2) - wheelInset), -((frameY / 2) - wheelInset))
        };

        public static final double absoluteConversion = 0;
        public static final double driveGearRatio = 6.75;
        public static final double angleGearRatio = 150.0 / 7.0;
        public static final double relativeConversion = 0;

        public static final double maxFreeSpeed = 4.60248;

        public static final class Simulation {
            public static final double driveMoi = 0.025; // Might not be perfect, taken from 6328's code
            public static final double angleMoi = 0.004096955; // Might not be perfect, taken from 6328's code
        }

        public static final class PoseEstimation {
            public static final int maxLogTicks = 50;
        }
    }

    public static final class Input {
        public static final double headingRateOfChange = 90;
    }
}
