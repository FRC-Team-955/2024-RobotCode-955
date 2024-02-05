package frc.robot.subsystem.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * The Shooter {@link Subsystem} for scoring in the speaker, amplifier, and trap
 */
public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;

    private final PIDController pivotPid;
    private final PIDController feedPid;
    private final PIDController feedFlywheelLeftPid;
    private final PIDController feedFlywheelRightPid;
    private final PIDController flywheelLeftPid;
    private final PIDController flywheelRightPid;

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



    private Shooter() {
        inputs = new ShooterIOInputsAutoLogged();
        io = Robot.isSimulation() ? new ShooterIOSim(inputs) : new ShooterIOSparkMax(inputs);

        pivotPid = new PIDController(0, 0, 0);
        feedPid = new PIDController(0, 0, 0);
        feedFlywheelLeftPid = new PIDController(0, 0, 0);
        feedFlywheelRightPid = new PIDController(0, 0, 0);
        flywheelLeftPid = new PIDController(0, 0, 0);
        flywheelRightPid = new PIDController(0, 0, 0);
    }



    @Override
    public void periodic() {

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

        io.setFeedVolts(feedPid.calculate(notePosition, noteSetpoint));
        if (flywheelIndexing) {
            io.setFlywheelLeftVolts(feedFlywheelLeftPid.calculate(notePosition, noteSetpoint));
            io.setFlywheelRightVolts(feedFlywheelRightPid.calculate(notePosition, noteSetpoint));
        }
        else {
            io.setFlywheelRightVolts(flywheelLeftPid.calculate(inputs.flywheelVelocityLeft, flywheelSetpointLeft));
            io.setFlywheelRightVolts(flywheelRightPid.calculate(inputs.flywheelVelocityRight, flywheelSetpointRight));
        }

        io.setPivotVolts(pivotPid.calculate(inputs.pivotPosition, pivotSetpoint));

        feedLast = inputs.feedPosition;
        flywheelLeftLast = inputs.flywheelPositionLeft;
        flywheelRightLast = inputs.flywheelPositionRight;
    }


    /**
     * Sets the target position for the shooter pivot
     * @param degrees The target position of the pivot in degrees from 0 to {@value Constants.Shooter#maxAngle}
     */
    public void setPivotPosition(double degrees) {
        pivotSetpoint = degrees;
    }

    /**
     * Sets the target position of the note inside the shooter
     * @param inches The position of the top of the note from the feed end of the shooter in inches
     */
    public void setNotePosition(double inches) {
        noteSetpoint = Math.max(inches, Constants.Shooter.ContactRanges.minSafe);
    }

    /**
     * Sets the target position of the note inside the shooter,
     * which gets limited to ensure that the note does not touch the flywheels
     * @param inches The position of the top of the note from the feed end of the shooter in inches
     */
    public void setNotePositionSafe(double inches) {
        noteSetpoint = MathUtil.clamp(inches,
                Constants.Shooter.ContactRanges.minSafe, Constants.Shooter.ContactRanges.maxSafe);
    }

    /**
     * Set the target velocity of the flywheels
     * @param meters The linear speed of the outer rim of the flywheels in meters per second
     */
    public void setFlywheelVelocity(double meters) {
        setFlywheelVelocityLeft(meters);
        setFlywheelVelocityRight(meters);
    }

    /**
     * Set the target velocity of the left flywheel
     * @param meters The linear speed of the outer rim of the flywheel in meters per second
     */
    public void setFlywheelVelocityLeft(double meters) {
        flywheelSetpointLeft = meters;
    }
    /**
     * Set the target velocity of the right flywheel
     * @param meters The linear speed of the outer rim of the flywheel in meters per second
     */
    public void setFlywheelVelocityRight(double meters) {
        flywheelSetpointRight = meters;
    }

    /**
     * Sets whether the flywheels should be used to help with indexing.
     * This should be set to true when the note has to be pulled fully into the shooter without being shot
     * @param indexing Whether the flywheels should be indexing
     */
    public void setFlywheelIndexing(boolean indexing) {
        flywheelIndexing = indexing;
    }


    /**
     * Gets the position of the shooter pivot
     * @return The rotational position of the shooter pivot in degrees from 0 to {@value Constants.Shooter#maxAngle}
     */
    public double getPivotPosition() {
        return inputs.pivotPosition;
    }

    /**
     * Gets whether there is a note in the shooter
     * @return Whether a note has been detected
     */
    public boolean hasNote() {
        return hasNote;
    }

    /**
     * Gets the indexing position of the note inside the shooter
     * @return The position of the top of the note from the feed end of the shooter in inches
     */
    public double getNotePosition() {
        return notePosition;
    }

    /**
     * Gets the velocity of the flywheels
     * @return The average of the linear velocity of the outer rim between both flywheels in meters per second
     */
    public double getFlywheelVelocity() {
        return (inputs.flywheelVelocityLeft + inputs.flywheelVelocityRight) / 2;
    }

    /**
     * Gets the velocity of the left flywheel
     * @return The linear velocity of the outer rim of the flywheel in meters per second
     */
    public double getFlywheelVelocityLeft() {
        return inputs.flywheelVelocityLeft;
    }
    /**
     * Gets the velocity of the right flywheel
     * @return The linear velocity of the outer rim of the flywheel in meters per second
     */
    public double getFlywheelVelocityRight() {
        return inputs.flywheelVelocityRight;
    }


    /**
     * Gets whether the pivot is at the angle setpoint
     * @return Whether the pivot is within {@value Constants.Shooter.Tolerances#pivot} degrees of the setpoint
     */
    public boolean atPivotSetpoint() {
        return atPivotSetpoint(Constants.Shooter.Tolerances.pivot);
    }
    /**
     * Gets where the pivot is at the angle setpoint within a given tolerance
     * @param tolerance The setpoint error tolerance in degrees
     * @return Whether the pivot is within the given tolerance from the setpoint
     */
    public boolean atPivotSetpoint(double tolerance) {
        return Math.abs(pivotSetpoint - inputs.pivotPosition) <= tolerance;
    }

    /**
     * Gets whether the note is at the position setpoint
     * @return Whether the note is within {@value Constants.Shooter.Tolerances#feed} inches of the indexing setpoint
     */
    public boolean atNoteSetpoint() {
        return atNoteSetpoint(Constants.Shooter.Tolerances.feed);
    }
    /**
     * Gets whether the note is at the position setpoint within a given tolerance
     * @param tolerance The setpoint error tolerance in inches
     * @return Whether the note is within the given tolerance from the indexing setpoint
     */
    public boolean atNoteSetpoint(double tolerance) {
        return Math.abs(noteSetpoint - notePosition) <= tolerance;
    }

    /**
     * Gets whether the flywheels are both at their linear velocity setpoints
     * @return Whether the flywheels are both within {@value Constants.Shooter.Tolerances#flywheel} meters per second of their setpoints
     */
    public boolean atFlywheelSetpoint() {
        return atFlywheelSetpoint(Constants.Shooter.Tolerances.flywheel);
    }
    /**
     * Gets whether the flywheels are both at their linear velocity setpoints within a given tolerance
     * @param tolerance The setpoint error tolerance in inches
     * @return Whether the flywheels are within the given tolerance from their setpoints
     */
    public boolean atFlywheelSetpoint(double tolerance) {
        return atFlywheelLeftSetpoint(tolerance) && atFlywheelRightSetpoint(tolerance);
    }

    /**
     * Gets whether the left flywheel is at the linear velocity setpoint
     * @return Whether the left flywheel is within {@value Constants.Shooter.Tolerances#flywheel} meters per second from the setpoint
     */
    public boolean atFlywheelLeftSetpoint() {
        return atFlywheelLeftSetpoint(Constants.Shooter.Tolerances.flywheel);
    }
    /**
     * Gets whether the left flywheel is at the linear velocity setpoint within a given tolerance
     * @param tolerance The setpoint error tolerance in inches
     * @return Whether the left flywheel is within the given tolerance from the setpoints
     */
    public boolean atFlywheelLeftSetpoint(double tolerance) {
        return Math.abs(flywheelSetpointLeft - inputs.flywheelVelocityLeft) <= tolerance;
    }
    /**
     * Gets whether the right flywheel is at the linear velocity setpoint
     * @return Whether the right flywheel is within {@value Constants.Shooter.Tolerances#flywheel} meters per second from the setpoint
     */
    public boolean atFlywheelRightSetpoint() {
        return atFlywheelRightSetpoint(Constants.Shooter.Tolerances.flywheel);
    }
    /**
     * Gets whether the right flywheel is at the linear velocity setpoint within a given tolerance
     * @param tolerance The setpoint error tolerance in inches
     * @return Whether the right flywheel is within the given tolerance from the setpoints
     */
    public boolean atFlywheelRightSetpoint(double tolerance) {
        return Math.abs(flywheelSetpointRight - inputs.flywheelVelocityRight) <= tolerance;
    }
}
