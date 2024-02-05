package frc.robot.subsystem.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

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



    public void setPivotPosition(double degrees) {
        pivotSetpoint = degrees;
    }

    public void setNotePosition(double inches) {
        noteSetpoint = Math.max(inches, Constants.Shooter.ContactRanges.minSafe);
    }

    public void setNotePositionSafe(double inches) {
        noteSetpoint = MathUtil.clamp(inches,
                Constants.Shooter.ContactRanges.minSafe, Constants.Shooter.ContactRanges.maxSafe);
    }

    public void setFlywheelVelocity(double meters) {
        setFlywheelVelocityLeft(meters);
        setFlywheelVelocityRight(meters);
    }

    public void setFlywheelVelocityLeft(double meters) {
        flywheelSetpointLeft = meters;
    }
    public void setFlywheelVelocityRight(double meters) {
        flywheelSetpointRight = meters;
    }

    public void setFlywheelIndexing(boolean indexing) {
        flywheelIndexing = indexing;
    }



    public double getPivotPosition() {
        return inputs.pivotPosition;
    }

    public boolean hasNote() {
        return hasNote;
    }
    public double getNotePosition() {
        return notePosition;
    }

    public double getFlywheelVelocity() {
        return (inputs.flywheelVelocityLeft + inputs.flywheelVelocityRight) / 2;
    }

    public double getFlywheelVelocityLeft() {
        return inputs.flywheelVelocityLeft;
    }
    public double getFlywheelVelocityRight() {
        return inputs.flywheelVelocityRight;
    }



    public boolean atPivotSetpoint() {
        return atPivotSetpoint(Constants.Shooter.Tolerances.pivot);
    }
    public boolean atPivotSetpoint(double tolerance) {
        return Math.abs(pivotSetpoint - inputs.pivotPosition) <= tolerance;
    }

    public boolean atNoteSetpoint() {
        return atNoteSetpoint(Constants.Shooter.Tolerances.feed);
    }
    public boolean atNoteSetpoint(double tolerance) {
        return Math.abs(noteSetpoint - notePosition) <= tolerance;
    }

    public boolean atFlywheelSetpoint() {
        return atFlywheelSetpoint(Constants.Shooter.Tolerances.flywheel);
    }
    public boolean atFlywheelSetpoint(double tolerance) {
        return atFlywheelLeftSetpoint(tolerance) && atFlywheelRightSetpoint(tolerance);
    }

    public boolean atFlywheelLeftSetpoint() {
        return atFlywheelLeftSetpoint(Constants.Shooter.Tolerances.flywheel);
    }
    public boolean atFlywheelLeftSetpoint(double tolerance) {
        return Math.abs(flywheelSetpointLeft - inputs.flywheelVelocityLeft) <= tolerance;
    }
    public boolean atFlywheelRightSetpoint() {
        return atFlywheelRightSetpoint(Constants.Shooter.Tolerances.flywheel);
    }
    public boolean atFlywheelRightSetpoint(double tolerance) {
        return Math.abs(flywheelSetpointRight - inputs.flywheelVelocityRight) <= tolerance;
    }
}
