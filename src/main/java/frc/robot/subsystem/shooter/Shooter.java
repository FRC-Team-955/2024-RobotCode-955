package frc.robot.subsystem.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

/**
 * The Shooter {@link Subsystem} for scoring in the speaker, amplifier, and trap
 */
public class Shooter extends SubsystemBase {

    public static Shooter instance;

    private final ShooterIO io;
    private final ShooterIOInputs inputs;

    private final ArmFeedforward pivotFf;
    private final SimpleMotorFeedforward feedFf;
    private final SimpleMotorFeedforward flywheelFf;

    private final TrapezoidProfile pivotMotionProfile;

    private double pivotSetpoint = Constants.Shooter.Setpoints.tuck;
    private double feedSetpoint = 0;
    private double flywheelSetpoint = 0;

    private Shooter() {
        instance = this;
        inputs = new ShooterIOInputs();
        io = Robot.isSimulation() ? new ShooterIOSim(inputs) : new ShooterIOSparkMax(inputs);

        pivotFf = new ArmFeedforward(Constants.Shooter.Control.Pivot.ks, Constants.Shooter.Control.Pivot.kg,
                Constants.Shooter.Control.Pivot.kv, Constants.Shooter.Control.Pivot.ka);
        feedFf = new SimpleMotorFeedforward(Constants.Shooter.Control.Feed.ks,
                Constants.Shooter.Control.Feed.kv, Constants.Shooter.Control.Feed.ka);
        flywheelFf = new SimpleMotorFeedforward(Constants.Shooter.Control.Flywheel.ks,
                Constants.Shooter.Control.Flywheel.kv, Constants.Shooter.Control.Flywheel.ka);

        pivotMotionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Constants.Shooter.Control.Pivot.maxVelocity, Constants.Shooter.Control.Pivot.maxAcceleration));
    }
    /**
     * Initialize the subsystem
     */
    public static void init() {
        if (instance == null) new Shooter();
    }



    public void updateInputs() {
        io.updateInputs();
//        Logger.processInputs("Shooter", inputs);
    }

    @Override
    public void periodic() {
        TrapezoidProfile.State pivotMotionSetpoint = pivotMotionProfile.calculate(Constants.loopTime,
                new TrapezoidProfile.State(inputs.pivotPosition, inputs.pivotVelocity),
                new TrapezoidProfile.State(pivotSetpoint, 0));
        io.updatePivotController(pivotMotionSetpoint.position, pivotFf.calculate(
                Units.Radians.convertFrom(pivotMotionSetpoint.position, Units.Degrees),
                Units.Radians.convertFrom(pivotMotionSetpoint.velocity, Units.Degrees)));
        io.updateFeedController(feedSetpoint, feedFf.calculate(feedSetpoint));
        io.updateFlywheelController(flywheelSetpoint, flywheelFf.calculate(flywheelSetpoint, 0),
                flywheelFf.calculate(flywheelSetpoint, 0));

        io.updateApplications();
        inputs.pivotPositionSetpoint = pivotSetpoint;
        inputs.feedVelocitySetpoint = feedSetpoint;
        inputs.flywheelVelocitySetpoint = flywheelSetpoint;
        Logger.processInputs("Shooter", inputs);
    }



    public static void setPivotPositionSepoint(double degrees) {
        instance.setPivotPositionSetpointI(degrees);
    }
    private void setPivotPositionSetpointI(double degrees) {
        pivotSetpoint = MathUtil.clamp(degrees,
                Constants.Shooter.Setpoints.min, Constants.Shooter.Setpoints.max);
    }

    public static void setFeedVelocitySetpoint(double metersPerSecond) {
        instance.setFeedVelocitySetpointI(metersPerSecond);
    }
    private void setFeedVelocitySetpointI(double metersPerSecond) {
        feedSetpoint = metersPerSecond;
    }

    public static void setFlywheelVelocitySetpoint(double metersPerSecond) {
        instance.setFlywheelVelocitySetpointI(metersPerSecond);
    }
    private void setFlywheelVelocitySetpointI(double metersPerSecond) {
        flywheelSetpoint = metersPerSecond;
    }


    /**
     * Gets whether the robot is safe to go under the stage
     * @return Whether the shooter is in the stowed position
     */
    public static boolean isStageSafe() {
        return instance.getPivotPositionI() < Constants.Shooter.Tolerances.pivot;
    }
    /**
     * Gets the position of the shooter pivot
     * @return The rotational position of the shooter pivot in degrees from 0 to {@value Constants.Shooter.Setpoints#max}
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
        return inputs.beamBreak;
    }

    /**
     * Gets the velocity of the flywheels
     * @return The average of the linear velocity of the outer rim between both flywheels in meters per second
     */
    public static double getFlywheelVelocity() {
        return instance.getFlywheelVelocityI();
    }
    private double getFlywheelVelocityI() {
        return (Math.abs(inputs.flywheelVelocityTop) + Math.abs(inputs.flywheelVelocityBottom)) / 2;
    }



    /**
     * Gets whether the pivot is at the angle setpoint
     * @return Whether the pivot is within {@value Constants.ShooterV1.Tolerances#pivot} degrees of the setpoint
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
}
