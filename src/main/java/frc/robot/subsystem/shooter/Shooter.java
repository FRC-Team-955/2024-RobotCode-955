package frc.robot.subsystem.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.conversion.AngleUtil;
import org.littletonrobotics.junction.Logger;

/**
 * The Shooter {@link Subsystem} for scoring in the speaker, amplifier, and trap
 */
public class Shooter extends SubsystemBase {

    public static Shooter instance;

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;

    private final PIDController pivotPid;
    private final PIDController feedPid;
    private final ArmFeedforward pivotFf;

    private double pivotSetpoint = 0;
    private double noteSetpoint = 0;
    private double flywheelSetpointLeft = 0;
    private double flywheelSetpointRight = 0;
    private boolean flywheelIndexing = false;
    private boolean hasNote = false;
    private double notePosition = 0;
    private double feedLast = 0;
    private double flywheelLeftLast = 0;
    private double flywheelRightLast = 0;



    public Shooter() {
        instance = this;
        inputs = new ShooterIOInputsAutoLogged();
        io = Robot.isSimulation() ? new ShooterIOSim(inputs) : new ShooterIOSparkMax(inputs);

        pivotPid = new PIDController(Constants.Shooter.Control.pivotKp, Constants.Shooter.Control.pivotKi,
                Constants.Shooter.Control.pivotKd);
        feedPid = new PIDController(Constants.Shooter.Control.feedKp, Constants.Shooter.Control.feedKi,
                Constants.Shooter.Control.feedKd);
        pivotFf = new ArmFeedforward(Constants.Shooter.Control.pivotKs, Constants.Shooter.Control.pivotKg,
                Constants.Shooter.Control.pivotKv, Constants.Shooter.Control.pivotKa);
    }
    /**
     * Initialize the subsystem
     */
    public static void init() {
        if (instance == null) new Shooter();
    }



    public void updateInputs() {
        io.updateInputs();
    }

    @Override
    public void periodic() {

        inputs.pivotPositionSetpoint = pivotSetpoint;

        if (hasNote) {
            if (notePosition > Constants.Shooter.ContactRanges.feedStart &&
                    notePosition < Constants.Shooter.ContactRanges.feedEnd) {
                notePosition += inputs.feedPosition - feedLast;
            }
            else if (notePosition > Constants.Shooter.ContactRanges.flywheelStart &&
                    notePosition < Constants.Shooter.ContactRanges.flywheelEnd) {
                notePosition += (inputs.flywheelPositionLeft - flywheelLeftLast +
                        inputs.flywheelPositionRight - flywheelRightLast) / 2;
            }

            if (notePosition > Constants.Shooter.ContactRanges.exit)
                hasNote = false;
        }
        else if (inputs.ultrasonicRange < Constants.Shooter.UltrasonicRanges.edge) {
            notePosition = Constants.Shooter.ultrasonicPosition;
            hasNote = true;
        }

//        double feedSpeed = MathUtil.clamp(feedPid.calculate(notePosition, noteSetpoint),
//                Constants.Shooter.FeedVelocities.feedMax, -Constants.Shooter.FeedVelocities.feedMax);

        double feedSpeed = noteSetpoint;

//        io.setFeedVolts(12 * feedSpeed * Constants.Shooter.FeedVelocities.feedMax);
        io.setFeedVolts(12 * feedSpeed);
        if (flywheelIndexing) {
            io.setFlywheelLeftVolts(12 * feedSpeed / Constants.Shooter.FeedVelocities.flywheelMax);
            io.setFlywheelRightVolts(12 * feedSpeed / Constants.Shooter.FeedVelocities.flywheelMax);
        }
        else {
            io.setFlywheelLeftVolts(12 * flywheelSetpointLeft / Constants.Shooter.FlywheelVelocities.max);
            io.setFlywheelRightVolts(12 * flywheelSetpointRight / Constants.Shooter.FlywheelVelocities.max);
        }

        io.setPivotVolts(pivotPid.calculate(inputs.pivotPosition, pivotSetpoint) +
                pivotFf.calculate(AngleUtil.degToRad(inputs.pivotPosition +
                        Constants.Shooter.Control.comAngleCompensation), AngleUtil.degToRad(inputs.pivotVelocity)));

        feedLast = inputs.feedPosition;
        flywheelLeftLast = inputs.flywheelPositionLeft;
        flywheelRightLast = inputs.flywheelPositionRight;

        Logger.processInputs("Shooter", inputs);
    }



    /**
     * Sets the target position of the shooter pivot to {@value Constants.Shooter.Setpoints#tuck}
     */
    public static void setPivotPositionTuck() {
        instance.setPivotPositionI(Constants.Shooter.Setpoints.tuck);
    }
    /**
     * Sets the target position of the shooter pivot to {@value Constants.Shooter.Setpoints#load}
     */
    public static void setPivotPositionLoad() {
        instance.setPivotPositionI(Constants.Shooter.Setpoints.load);
    }
    /**
     * Sets the target position of the shooter pivot to {@value Constants.Shooter.Setpoints#subwoofer}
     */
    public static void setPivotPositionSubwoofer() {
        instance.setPivotPositionI(Constants.Shooter.Setpoints.subwoofer);
    }
    /**
     * Sets the target position of the shooter pivot to {@value Constants.Shooter.Setpoints#amp}
     */
    public static void setPivotPositionAmp() {
        instance.setPivotPositionI(Constants.Shooter.Setpoints.amp);
    }
    /**
     * Sets the target position of the shooter pivot to {@value Constants.Shooter.Setpoints#trap}
     */
    public static void setPivotPositionTrap() {
        instance.setPivotPositionI(Constants.Shooter.Setpoints.trap);
    }
    /**
     * Sets the target position for the shooter pivot
     * @param degrees The target position of the pivot in degrees from 0 to {@value Constants.Shooter#maxAngle}
     */
    public static void setPivotPosition(double degrees) {
        instance.setPivotPositionI(degrees);
    }
    private void setPivotPositionI(double degrees) {
        pivotSetpoint = degrees;
    }

    /**
     * Sets the target position of the note inside the shooter to {@value Constants.Shooter.ContactRanges#minSafe}
     */
    public static void setNotePositionMinSafe() {
        instance.setNotePositionI(Constants.Shooter.ContactRanges.minSafe);
    }
    /**
     * Sets the target position of the note inside the shooter to {@value Constants.Shooter.ContactRanges#held}
     */
    public static void setNotePositionHeld() {
        instance.setNotePositionI(Constants.Shooter.ContactRanges.held);
    }
    /**
     * Sets the target position of the note inside the shooter to {@value Constants.Shooter.ContactRanges#maxSafe}
     */
    public static void setNotePositionMaxSafe() {
        instance.setNotePositionI(Constants.Shooter.ContactRanges.maxSafe);
    }
    /**
     * Sets the target position of the note inside the shooter to {@value Constants.Shooter.ContactRanges#exit}
     */
    public static void setNotePositionExit() {
        instance.setNotePositionI(Constants.Shooter.ContactRanges.exit);
    }
    /**
     * Sets the target position of the note inside the shooter,
     * which gets limited to ensure that the note does not touch the flywheels
     * @param inches The position of the top of the note from the feed end of the shooter in inches
     */
    public static void setNotePositionSafe(double inches) {
        instance.setNotePositionI(MathUtil.clamp(inches, Constants.Shooter.ContactRanges.minSafe,
                Constants.Shooter.ContactRanges.maxSafe));
    }
    /**
     * Sets the target position of the note inside the shooter
     * @param inches The position of the top of the note from the feed end of the shooter in inches
     */
    public static void setNotePosition(double inches) {
        instance.setNotePositionI(inches);
    }
    private void setNotePositionI(double inches) {
//        noteSetpoint = Math.max(inches, Constants.Shooter.ContactRanges.minSafe);
        noteSetpoint = inches;
    }

    /**
     * Sets the target velocity of the flywheels to 0
     */
    public static void setFlywheelVelocityZero() {
        instance.setFlywheelVelocityLeftI(0);
        instance.setFlywheelVelocityRightI(0);
    }
    /**
     * Sets the target velocity of the flywheels to {@value Constants.Shooter.FlywheelVelocities#subwoofer}
     */
    public static void setFlywheelVelocitySubwoofer() {
        instance.setFlywheelVelocityLeftI(Constants.Shooter.FlywheelVelocities.subwoofer);
        instance.setFlywheelVelocityRightI(Constants.Shooter.FlywheelVelocities.subwoofer);
    }
    /**
     * Sets the target velocity of the flywheels to {@value Constants.Shooter.FlywheelVelocities#max}
     */
    public static void setFlywheelVelocityMax() {
        instance.setFlywheelVelocityLeftI(Constants.Shooter.FlywheelVelocities.max);
        instance.setFlywheelVelocityRightI(Constants.Shooter.FlywheelVelocities.max);
    }
    /**
     * Sets the target velocity of the flywheels
     * @param meters The linear speed of the outer rim of the flywheels in meters per second
     */
    public static void setFlywheelVelocity(double meters) {
        instance.setFlywheelVelocityLeftI(meters);
        instance.setFlywheelVelocityRightI(meters);
    }

    /**
     * Sets the target velocity of the left flywheel
     * @param meters The linear speed of the outer rim of the flywheel in meters per second
     */
    public static void setFlywheelVelocityLeft(double meters) {
        instance.setFlywheelVelocityLeftI(meters);
    }
    private void setFlywheelVelocityLeftI(double meters) {
        flywheelSetpointLeft = meters;
    }
    /**
     * Sets the target velocity of the right flywheel
     * @param meters The linear speed of the outer rim of the flywheel in meters per second
     */
    public static void setFlywheelVelocityRight(double meters) {
        instance.setFlywheelVelocityRightI(meters);
    }
    private void setFlywheelVelocityRightI(double meters) {
        flywheelSetpointRight = meters;
    }

    /**
     * Sets whether the flywheels should be used to help with indexing.
     * This should be set to true when the note has to be pulled fully into the shooter without being shot
     * @param indexing Whether the flywheels should be indexing
     */
    public static void setFlywheelIndexing(boolean indexing) {
        instance.setFlywheelIndexingI(indexing);
    }
    private void setFlywheelIndexingI(boolean indexing) {
        io.setFlywheelBrake(indexing);
        flywheelIndexing = indexing;
    }


    /**
     * Gets whether the robot is safe to go under the stage
     * @return Whether the shooter is in the stowed position
     */
    public static boolean isStageSafe() {
        return instance.getPivotPositionI() < Constants.Shooter.Setpoints.maxSafe;
    }
    /**
     * Gets the position of the shooter pivot
     * @return The rotational position of the shooter pivot in degrees from 0 to {@value Constants.Shooter#maxAngle}
     */
    public static double getPivotPosition() {
        return instance.getPivotPositionI();
    }
    private double getPivotPositionI() {
        return inputs.pivotPosition;
    }

    /**
     * Gets whether there is a note in the shooter
     * @return Whether a note has been detected
     */
    public static boolean hasNote() {
        return instance.hasNoteI();
    }
    private boolean hasNoteI() {
        return hasNote;
    }

    /**
     * Gets the indexing position of the note inside the shooter
     * @return The position of the top of the note from the feed end of the shooter in inches
     */
    public static double getNotePosition() {
        return instance.getNotePositionI();
    }
    private double getNotePositionI() {
        return notePosition;
    }

    /**
     * Gets the velocity of the flywheels
     * @return The average of the linear velocity of the outer rim between both flywheels in meters per second
     */
    public static double getFlywheelVelocity() {
        return instance.getFlywheelVelocityI();
    }
    private double getFlywheelVelocityI() {
        return (inputs.flywheelVelocityLeft + inputs.flywheelVelocityRight) / 2;
    }

    /**
     * Gets the velocity of the left flywheel
     * @return The linear velocity of the outer rim of the flywheel in meters per second
     */
    public static double getFlywheelVelocityLeft() {
        return instance.getFlywheelVelocityLeftI();
    }
    private double getFlywheelVelocityLeftI() {
        return inputs.flywheelVelocityLeft;
    }
    /**
     * Gets the velocity of the right flywheel
     * @return The linear velocity of the outer rim of the flywheel in meters per second
     */
    public static double getFlywheelVelocityRight() {
        return instance.getFlywheelVelocityRightI();
    }
    private double getFlywheelVelocityRightI() {
        return inputs.flywheelVelocityRight;
    }



    /**
     * Gets whether the pivot is at the angle setpoint
     * @return Whether the pivot is within {@value Constants.Shooter.Tolerances#pivot} degrees of the setpoint
     */
    public static boolean atPivotSetpoint() {
        return instance.atPivotSetpointI(Constants.Shooter.Tolerances.pivot);
    }
    /**
     * Gets where the pivot is at the angle setpoint within a given tolerance
     * @param tolerance The setpoint error tolerance in degrees
     * @return Whether the pivot is within the given tolerance from the setpoint
     */
    public static boolean atPivotSetpoint(double tolerance) {
        return instance.atPivotSetpointI(tolerance);
    }
    private boolean atPivotSetpointI(double tolerance) {
        return Math.abs(pivotSetpoint - inputs.pivotPosition) <= tolerance;
    }

    /**
     * Gets whether the note is at the position setpoint
     * @return Whether the note is within {@value Constants.Shooter.Tolerances#indexing} inches of the indexing setpoint
     */
    public static boolean atNoteSetpoint() {
        return instance.atNoteSetpointI(Constants.Shooter.Tolerances.indexing);
    }
    /**
     * Gets whether the note is at the position setpoint within a given tolerance
     * @param tolerance The setpoint error tolerance in inches
     * @return Whether the note is within the given tolerance from the indexing setpoint
     */
    public static boolean atNoteSetpoint(double tolerance) {
        return instance.atNoteSetpointI(tolerance);
    }
    private boolean atNoteSetpointI(double tolerance) {
        return inputs.ultrasonicRange <= Constants.Intake.UltrasonicRanges.noteCaptureDistance;
//        return hasNote && Math.abs(noteSetpoint - notePosition) <= tolerance;
    }

    /**
     * Gets whether the flywheels are both at their linear velocity setpoints
     * @return Whether the flywheels are both within {@value Constants.Shooter.Tolerances#flywheel} meters per second of their setpoints
     */
    public static boolean atFlywheelSetpoint() {
        return instance.atFlywheelSetpointI(Constants.Shooter.Tolerances.flywheel);
    }
    /**
     * Gets whether the flywheels are both at their linear velocity setpoints within a given tolerance
     * @param tolerance The setpoint error tolerance in inches
     * @return Whether the flywheels are within the given tolerance from their setpoints
     */
    public static boolean atFlywheelSetpoint(double tolerance) {
        return instance.atFlywheelSetpointI(tolerance);
    }
    private boolean atFlywheelSetpointI(double tolerance) {
        return atFlywheelLeftSetpoint(tolerance) && atFlywheelRightSetpoint(tolerance);
    }

    /**
     * Gets whether the left flywheel is at the linear velocity setpoint
     * @return Whether the left flywheel is within {@value Constants.Shooter.Tolerances#flywheel} meters per second from the setpoint
     */
    public static boolean atFlywheelLeftSetpoint() {
        return instance.atFlywheelLeftSetpointI(Constants.Shooter.Tolerances.flywheel);
    }
    /**
     * Gets whether the left flywheel is at the linear velocity setpoint within a given tolerance
     * @param tolerance The setpoint error tolerance in inches
     * @return Whether the left flywheel is within the given tolerance from the setpoints
     */
    public static boolean atFlywheelLeftSetpoint(double tolerance) {
        return instance.atFlywheelLeftSetpointI(tolerance);
    }
    private boolean atFlywheelLeftSetpointI(double tolerance) {
        return Math.abs(flywheelSetpointLeft - inputs.flywheelVelocityLeft) <= tolerance;
    }

    /**
     * Gets whether the right flywheel is at the linear velocity setpoint
     * @return Whether the right flywheel is within {@value Constants.Shooter.Tolerances#flywheel} meters per second from the setpoint
     */
    public static boolean atFlywheelRightSetpoint() {
        return instance.atFlywheelRightSetpointI(Constants.Shooter.Tolerances.flywheel);
    }
    /**
     * Gets whether the right flywheel is at the linear velocity setpoint within a given tolerance
     * @param tolerance The setpoint error tolerance in inches
     * @return Whether the right flywheel is within the given tolerance from the setpoints
     */
    public static boolean atFlywheelRightSetpoint(double tolerance) {
        return instance.atFlywheelRightSetpointI(tolerance);
    }
    private boolean atFlywheelRightSetpointI(double tolerance) {
        return Math.abs(flywheelSetpointRight - inputs.flywheelVelocityRight) <= tolerance;
    }
}
