package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Map;
import frc.robot.subsystem.swerve.SwerveMod;

public class Constants {

    /** The width of the robot frame in meters **/
    public static final double frameX = 0.6858;

    /** The length of the robot frame in meters **/
    public static final double frameY = 0.6858;

    /** The time in seconds between each code loop **/
    public static final double loopTime = 0.02;

    public static final class Shooter {
        public static final class Setpoints {
            public static final double tuck = 0;
            public static final double load = 37;
            public static final double hover = 60;
            public static final double subwoofer = 37; // TODO
            public static final double amp = 117; // TODO
            public static final double trap = 20; // TODO
            public static final double source = 70;
            public static final double max = 150;

            public static final double autoShootShort = 15;
            public static final double autoShootLong = 15;
        }
        public static final class Voltages {
            public static final double feedUp = 3;
        }
        public static final class Control {
            public static final double kp = 0.1; // TODO
            public static final double ki = 0; // TODO
            public static final double kd = 0.011; // TODO
            public static final double kg = 0.59; // TODO
            public static final double kv = 0;
            public static final double ka = 0;
            public static final double ks = 0;
            public static final double comAngleCompensation = -15; // TODO
        }
        public static final class Tolerances {
            public static final double pivot = 5;
        }
        public static final class GearRatios {
            public static final double pivot = 1 / 40.0;
            public static final double feed = 1;
            public static final double flywheel = 1;
        }
        public static final class Ids {
            public static final int pivot = 7;
            public static final int feed = 9;
            public static final int flywheelTop = 10;
            public static final int flywheelBottom = 8;
            public static final int beamBreak = 6;
        }

        //Add values of for distance, then angle
        public static final Map<Double, Double> interpolationMap = Map.of(
                2.3876, 60.0,
                3.556, 70.0,
                5.3594, 76.0
        );
    }

    /** Constants relating to the Intake **/
    public static final class Intake {
        /** The gear reduction from the deploy motor to the pivot **/
        public static final double gearRatioDeploy = 1.0 / 45.0;

        /** The gear reduction from the intake motor to the intake rollers **/
        public static final double gearRatioIntake = 1.0 / 75.0;

        /** The angle at which the intake leaves the frame perimeter **/
        public static final double extrusionThreshold = 90; // TODO

        /** The max deploy angle **/
        public static final double maxAngle = 180;

        /** The default error tolerance for setpoints in degrees **/
        public static final double tolerance = 5;

        public static final int limitSwitchDenoiseLength = 25;

        /** CAN ID for the deploy motor **/
        public static final int deployId = 3;

        /** CAN ID for the intake motor **/
        public static final int intakeId = 16;

        /** Serial ID for the ultrasonic sensor ping channel **/
        public static final int limitSwitchId = 4; // TODO

        /** The setpoints for the intake deployment pivot **/
        public static final class Setpoints {
            /** Able to give note to the shooter **/
            public static final double handoff = 10;

            /** In the frame but out of the shooter's way **/
            public static final double hover = 60;

            /** Able to intake from the ground **/
            public static final double intake = 175;

            public static final double start = 17;
        }

        /** Percentages for the intake motor **/
        public static final class Percents {
            /** Intake the note from the ground **/
            public static final double intake = 0.1;

            /** Keep the note in the intake **/
            public static final double hold = 0;

            /** Pass the note to the shooter **/
            public static final double handoff = -0.25;
        }

        /** The ranges reported by the ultrasonic sensor for different note distances within the intake **/
        public static final class UltrasonicRanges {
            /** The note has been grabbed by the intake wheels **/
            public static final double noteCaptureDistance = 12.0;

        }

        /** Values for pid, feedforward, and other controllers **/
        public static final class Control {
            /** Extend PID kp **/
            public static final double kp = 0.17; // TODO
            /** Extend PID ki **/
            public static final double ki = 0; // TODO
            /** Extend PID kd **/
            public static final double kd = 0.003; // TODO
            /** Extend FF kg **/
            public static final double kg = 0.36; // TODO
            /** Extend FF kv **/
            public static final double kv = 0; // TODO
            /** Extend FF ka **/
            public static final double ka = 0; // TODO
            /** Extend FF ks **/
            public static final double ks = 0; // TODO
            /** The angle the FF input should be offset by to account for the COM not being at the exact angle of the
             * encoder reading **/
            public static final double comAngleCompensation = 30;
        }

        /** Values needed for simulation **/
        public static final class Simulation {
            /** The moment of inertia of the intake in kg m^2 **/
            public static final double moi = 0.1818003362;

            /** The length of the intake in meters **/
            public static final double length = 0.36;
        }
    }

    /** Constants relating to handoffs **/
    public static final class Handoff {
        /** The range in which a note sticking out of the intake or shooter will contact both in degrees **/
        public static final double contactRange = 20;
    }

    /** Constants relating the Climber **/
    public static final class Climber {
        /** The gear ratio from the motor to the climber **/
        public static final double gearRatio = 1; // TODO

        /** The radius of the mechanism applying force to the telescoping tube **/
        public static final double radius = 1; // TODO

        /** The max extension length of the climber **/
        public static final double extensionLength = 1; // TODO

        /** The voltage to be applied for raising the climber up (Not climbing) **/
        public static final double extendVoltage = 4.0; // TODO

        /** The voltage to be applied for pulling the climber down (Climbing) **/
        public static final double climbVoltage = 12.0; // TODO

        /** CAN ID for the left climber motor **/
        public static final int leftId = 4;

        /** CAN ID for the right climber motor **/
        public static final int rightId = 13;
    }

    /** Constants relating to the swerve drivebase **/
    public static final class Swerve {

        /** The radius of the wheel in meters **/
        public static final double wheelRadius = 0.0508;

        /** How far back the wheel is set in from the frame **/
        public static final double wheelInset = 0.066675;

        /** CAN IDs for the drive motors for each respective swerve module as described by {@link SwerveMod#modId} **/
        public static final int[] driveIds = new int[] { 2, 14, 11, 5 };

        /** CAN IDs for the angle motors for each respective swerve module as described by {@link SwerveMod#modId} **/
        public static final int[] angleIds = new int[] { 1, 15, 12, 6 };

        /** CAN IDs for the CANCoders for each respective swerve module as described by {@link SwerveMod#modId} **/
        public static final int[] encoderIds = new int[] { 11, 12, 13, 14 };

        /** CAN ID for the Pigeon 2 gyro **/
        public static final int gyroId = 10;

        /** {@link Translation2d}s representing the positions for each respective swerve wheel relative to the robot center as described by {@link SwerveMod#modId} **/
        public static final Translation2d[] modulePositions = new Translation2d[] {
                new Translation2d( ((frameX / 2) - wheelInset),  ((frameY / 2) - wheelInset)),
                new Translation2d( ((frameX / 2) - wheelInset), -((frameY / 2) - wheelInset)),
                new Translation2d(-((frameX / 2) - wheelInset), -((frameY / 2) - wheelInset)),
                new Translation2d(-((frameX / 2) - wheelInset),  ((frameY / 2) - wheelInset))
        };

        /** The conversion from CANCoder units to degrees **/
        public static final double absoluteConversion = 360.0;

        /** The gear reduction from the drive motors to the wheels **/
        public static final double driveGearRatio = 1.0 / 6.75;

        /** The gear reduction from the angle motors to the wheels **/
        public static final double angleGearRatio = 7.0 / 150.0;

        /** The conversion from Neo encoder units to degrees **/
        public static final double relativeConversion = 360.0;

        /** The tolerances for checking if a setpoint has been reached **/
        public static final class Tolerances {
            /** Heading position tolerance in degrees **/
            public static double heading = 3;
        }

        /** The values relating to the physical limits of the robot **/
        public static final class Constraints {
            /** The max free speed of the robot in meters per second **/
            public static final double maxFreeSpeed = 3.707;

            /** The max acceleration of the robot in meters per second squared **/
            public static final double maxAcceleration = 6.000;

            /** The max rotational velocity in degrees per second **/
            public static final double maxRotationSpeed = 360;

            /** The max rotational acceleration in rotations per second squared **/
            public static final double maxRotationalAcceleration = 1;
        }

        /** Values needed for simulation **/
        public static final class Simulation {
            /** The moment of inertia of the drive wheel / robot **/
            public static final double driveMoi = 0.025; // Might not be perfect

            /** The moment of inertia of the angle wheel **/
            public static final double angleMoi = 0.004096955; // Might not be perfect, taken from 6328's code
        }
    }

    /** Constants relating to the shooter **/
    public static final class ShooterV1 {
        /** The gear reduction from the motor to the pivot **/
        public static final double gearRatioAngle = 1.0 / 40.0; // TODO

        /** The gear reduction from the motor to the feed wheels **/
        public static final double gearRatioFeed = 1; // TODO

        /** The radius of the feed wheels **/
        public static final double wheelRadiusFeed = 1; // TODO

        /** The radius of the flywheels **/
        public static final double wheelRadiusFlywheel = 1; // TODO

        /** The max angle of the pivot **/
        public static final double maxAngle = 235; // TODO

        /** The indexing position of the ultrasonic sensor **/
        public static final double ultrasonicPosition = 10;  // TODO

        /** CAN ID for the pivot motor **/
        public static final int pivotId = 7;

        /** CAN ID for the feed motor **/
        public static final int feedId = 8;

        /** CAN ID for the left flywheel motor **/
        public static final int flywheelLeftId = 10;

        /** CAN ID for the right flywheel motor **/
        public static final int flywheelRightId = 9;

        /** Serial ID for the ultrasonic sensor ping channel **/
        public static final int ultrasonicId = 0; // TODO

        /** Pivot angle setpoints **/
        public static final class Setpoints {
            /** Fully stowed and able to go under the stage **/
            public static final double tuck = 0;

            /** Biggest angle at which the robot is able to go under the stage **/
            public static final double maxSafe = 5; // TODO

            /** Able to load from the intake **/
            public static final double load = 35;

            /** Able to shoot from the subwoofer **/
            public static final double subwoofer = 35;

            /** Able to score in the amp **/
            public static final double amp = 135; // TODO

            /** Able to shoot into the trap **/
            public static final double trap = 20; // TODO
        }

        /** Flywheel velocity setpoints **/
        public static final class FlywheelVelocities {
            /** The max velocity of the shooter flywheels **/
            public static final double max = 10; // TODO

            /** The min velocity to shoot from the subwoofer **/
            public static final double subwoofer = 5; // TODO
        }

        /** Feed velocity values **/
        public static final class FeedVelocities {
            public static final double flywheelMax = 198.1297766868;
            public static final double feedMax = 22.6434030498;
        }

        /** The default setpoint tolerances for checking if a setpoint has been reached **/
        public static final class Tolerances {
            /** Pivot position tolerance in degrees **/
            public static final double pivot = 3;

            /** Indexing position tolerance in inches **/
            public static final double indexing = 0.5;

            /** Flywheel velocity tolerance in meters per second **/
            public static final double flywheel = 0.1;
        }

        /** The ranges reported by the ultrasonic sensor for different indexing positions **/
        public static final class UltrasonicRanges {
            /** The note is not indexed in front of the sensor **/
            public static final double empty = 12; // TODO

            /** The edge of the note is in front of the sensor **/
            public static final double edge = 6; // TODO
        }

        /** The indexing positions within which the note is contacted by different parts of the shooter **/
        public static final class ContactRanges {
            /** The first position touching the feed wheels **/
            public static final double feedStart = 0; // TODO

            /** The last position touching the feed wheels **/
            public static final double feedEnd = 16; // TODO

            /** The first position touching the flywheels **/
            public static final double flywheelStart = 12; // TODO

            /** The last position touching the flywheels **/
            public static final double flywheelEnd = 32; // TODO

            /** The last position in the shooter **/
            public static final double exit = 35; // TODO

            /** The first position where the note will not fall out **/
            public static final double minSafe = 5; // TODO

            /** The last position where the note will not touch the flywheels **/
            public static final double maxSafe = 14; // TODO

            /** The position where the note is fully in the shooter **/
            public static final double held = 14; // TODO
        }

        /** Values for pid, feedforward, and other controllers **/
        public static final class Control {
            /** Pivot PID kp **/
            public static final double pivotKp = 0.15; // TODO
            /** Pivot PID ki **/
            public static final double pivotKi = 0; // TODO
            /** Pivot PID kd **/
            public static final double pivotKd = 0.003; // TODO
            /** Pivot FF kg **/
            public static final double pivotKg = 0.475; // TODO
            /** Pivot FF kv **/
            public static final double pivotKv = 0; // TODO
            /** Pivot FF ka **/
            public static final double pivotKa = 0; // TODO
            /** Pivot FF ks **/
            public static final double pivotKs = 0; // TODO
            /** The angle the FF input should be offset by to account for the COM not being at the exact angle of the
             * encoder reading **/
            public static final double comAngleCompensation = -20.0; // TODO
            /** Feed position PID kp **/
            public static final double feedKp = 0.1; // TODO
            /** Feed position PID ki **/
            public static final double feedKi = 0; // TODO
            /** Feed position PID kd **/
            public static final double feedKd = 0; // TODO
            /** Flywheel PID kp **/
            public static final double flywheelKp = 0.1; // TODO
            /** Flywheel PID ki **/
            public static final double flywheelKi = 0; // TODO
            /** Flywheel PID kd **/
            public static final double flywheelKd = 0; // TODO
        }

        /** Values needed for simulation **/
        public static final class Simulation {
            /** The moment of inertia of the pivot in kg m^2 **/
            public static final double moi = 0.1; // TODO

            /** The length of the shooter in meters **/
            public static final double length = 0.3; // TODO
        }
    }

    public static final class Vision {
        public static final Transform3d distancePosition = new Transform3d(new Translation3d(-0.254, -0.1143, 0.3429),
                new Rotation3d(0, -Math.PI / 6.0, Math.PI));
        public static final Transform3d verticalPosition = new Transform3d(new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));
    }

    public static final class Stage {
        public static final class Safety {
            public static final double warningDistance = 1.5;
            public static final double dangerDistance = 0.6;
            public static final double warningTime = 1;
            public static final double dangerTime = 0.5;
        }
    }
}
