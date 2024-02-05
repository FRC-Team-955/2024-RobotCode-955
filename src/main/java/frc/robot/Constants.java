package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    public static final double frameX = 0.6858;
    public static final double frameY = 0.6858;

    public static final class Shooter {
        public static final double gearRatioAngle = 1; // TODO
        public static final double gearRatioFeed = 1; // TODO

        public static final double wheelRadius = 1; // TODO

        public static final double maxAngle = 235; // TODO

        public static final double ultrasonicPosition = 10;  // TODO

        public static final int pivotId = 21;
        public static final int feedId = 22;
        public static final int flywheelLeftId = 23;
        public static final int flywheelRightId = 24;

        public static final int pingId = 3; // TODO
        public static final int echoId = 4; // TODO

        public static final class Setpoints {
            public static final double tuck = 0;
            public static final double load = 35;
            public static final double amp = 235; // TODO
        }

        public static final class Tolerances {
            public static final double pivot = 3;
            public static final double feed = 0.5;
            public static final double flywheel = 0.1;
        }

        public static final class UltrasonicRanges {
            public static final double empty = 12; // TODO
            public static final double edge = 6; // TODO
        }

        public static final class ContactRanges {
            public static final double feedStart = 0; // TODO
            public static final double feedEnd = 16; // TODO
            public static final double flywheelStart = 12; // TODO
            public static final double flywheelEnd = 32; // TODO
            public static final double exit = 35; // TODO
            public static final double minSafe = 5;
            public static final double maxSafe = 14;
        }

        public static final class Simulation {
            public static final double moi = 0.1; // TODO
            public static final double length = 0.3; // TODO
        }
    }

    public static final class Intake {

        public static final double gearRatio = 75;
        public static final double extrusionThreshold = 90; // TODO

        public static final double maxAngle = 180;

        public static final double noteCaptureDistance = 12; // TODO
        public static final double noteSecureDistance = 3; // TODO

        public static final int deployId = 31;
        public static final int intakeId = 32;

        public static final int pingId = 1; // TODO
        public static final int echoId = 2; // TODO

        public static final class Simulation {
            public static final double moi = 0.1818003362;
            public static final double length = 0.36;
        }
    }

    public static final class Climber {
        public static final double gearRatio = 1; // TODO
        public static final double radius = 1; // TODO
        public static final double extensionLength = 1; // TODO

        public static final double extendVoltage = 4.0; // TODO
        public static final double climbVoltage = 12.0; // TODO

        public static final int leftId = 41;
        public static final int rightId = 42;
    }

    public static final class Swerve {

        // 2 Inches
        public static final double wheelRadius = 0.0508;
        // 2.625 inches
        public static final double wheelInset = 0.066675;

        public static final int[] driveIds = new int[] { 2, 4, 6, 8 };
        public static final int[] angleIds = new int[] { 1, 3, 5, 7 };
        public static final int[] encoderIds = new int[] { 11, 12, 13, 14 };

        public static final Translation2d[] modulePositions = new Translation2d[] {
                new Translation2d( ((frameX / 2) - wheelInset),  ((frameY / 2) - wheelInset)),
                new Translation2d(-((frameX / 2) - wheelInset),  ((frameY / 2) - wheelInset)),
                new Translation2d(-((frameX / 2) - wheelInset), -((frameY / 2) - wheelInset)),
                new Translation2d( ((frameX / 2) - wheelInset), -((frameY / 2) - wheelInset))
        };

        public static final double absoluteConversion = 360.0;
        public static final double driveGearRatio = 6.75;
        public static final double angleGearRatio = 150.0 / 7.0;
        public static final double relativeConversion = 1; // TODO

        public static final double maxFreeSpeed = 4.60248;
        public static final double maxRotationSpeed = 360;

        public static final class Simulation {
            public static final double driveMoi = 0.025; // Might not be perfect
            public static final double angleMoi = 0.004096955; // Might not be perfect, taken from 6328's code
        }
    }
}
