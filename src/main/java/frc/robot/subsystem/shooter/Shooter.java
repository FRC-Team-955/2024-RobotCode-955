package frc.robot.subsystem.shooter;

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
    private final ArmFeedforward pivotFf;

    private double pivotSetpoint = 0;

    private boolean hasNote = false;
    private boolean isIntaking = false;
    private boolean flywheelSpinup = false;
    private boolean isShooting = false;
    private int outCounter = 25;



    public Shooter() {
        instance = this;
        inputs = new ShooterIOInputsAutoLogged();
        io = Robot.isSimulation() ? new ShooterIOSim(inputs) : new ShooterIOSparkMax(inputs);

        pivotPid = new PIDController(Constants.ShooterV1.Control.pivotKp, Constants.ShooterV1.Control.pivotKi,
                Constants.ShooterV1.Control.pivotKd);
        pivotFf = new ArmFeedforward(Constants.ShooterV1.Control.pivotKs, Constants.ShooterV1.Control.pivotKg,
                Constants.ShooterV1.Control.pivotKv, Constants.ShooterV1.Control.pivotKa);
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

        if ((isIntaking || hasNote) && !inputs.beamBreak) {
            io.setFeedVolts(Constants.Shooter.Voltages.feedUp);
        }
        if (isIntaking && inputs.beamBreak) {
            hasNote = true;
            isIntaking = false;
        }
        if (isShooting || flywheelSpinup)
            io.setFlywheelVolts(12);
        if (isShooting)
            io.setFeedVolts(12);
        if (isShooting && inputs.beamBreak)
            outCounter = 25;
        else if (isShooting) {
            outCounter--;
            if (outCounter == 0) {
                isShooting = false;
                hasNote = false;
            }
        }

        io.setPivotVolts(pivotPid.calculate(inputs.pivotPosition, pivotSetpoint) +
                pivotFf.calculate(AngleUtil.degToRad(inputs.pivotPosition +
                        Constants.ShooterV1.Control.comAngleCompensation), AngleUtil.degToRad(inputs.pivotVelocity)));

        Logger.processInputs("Shooter", inputs);
    }



    /**
     * Sets the target position of the shooter pivot to {@value Constants.ShooterV1.Setpoints#tuck}
     */
    public static void setPivotPositionTuck() {
        instance.setPivotPositionI(Constants.ShooterV1.Setpoints.tuck);
    }
    /**
     * Sets the target position of the shooter pivot to {@value Constants.ShooterV1.Setpoints#load}
     */
    public static void setPivotPositionLoad() {
        instance.setPivotPositionI(Constants.ShooterV1.Setpoints.load);
    }
    /**
     * Sets the target position of the shooter pivot to {@value Constants.ShooterV1.Setpoints#subwoofer}
     */
    public static void setPivotPositionSubwoofer() {
        instance.setPivotPositionI(Constants.ShooterV1.Setpoints.subwoofer);
    }
    /**
     * Sets the target position of the shooter pivot to {@value Constants.ShooterV1.Setpoints#amp}
     */
    public static void setPivotPositionAmp() {
        instance.setPivotPositionI(Constants.ShooterV1.Setpoints.amp);
    }
    /**
     * Sets the target position of the shooter pivot to {@value Constants.ShooterV1.Setpoints#trap}
     */
    public static void setPivotPositionTrap() {
        instance.setPivotPositionI(Constants.ShooterV1.Setpoints.trap);
    }
    /**
     * Sets the target position for the shooter pivot
     * @param degrees The target position of the pivot in degrees from 0 to {@value Constants.ShooterV1#maxAngle}
     */
    public static void setPivotPosition(double degrees) {
        instance.setPivotPositionI(degrees);
    }
    private void setPivotPositionI(double degrees) {
        pivotSetpoint = degrees;
    }

    public static void setIntaking(boolean intaking) {
        instance.setIntakingI(intaking);
    }
    private void setIntakingI(boolean intaking) {
        isIntaking = intaking;
    }
    public static void setSpinup(boolean spinup) {
        instance.setSpinupI(spinup);
    }
    private void setSpinupI(boolean spinup) {
        flywheelSpinup = spinup;
    }
    public static void shoot() {
        instance.shootI();
    }
    private void shootI() {
        isShooting = true;
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
     * @return The rotational position of the shooter pivot in degrees from 0 to {@value Constants.ShooterV1#maxAngle}
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
     * Gets the velocity of the flywheels
     * @return The average of the linear velocity of the outer rim between both flywheels in meters per second
     */
    public static double getFlywheelVelocity() {
        return instance.getFlywheelVelocityI();
    }
    private double getFlywheelVelocityI() {
        return (inputs.flywheelVelocityTop + inputs.flywheelVelocityBottom) / 2;
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
