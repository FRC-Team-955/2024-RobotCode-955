package frc.robot.subsystem.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    private final PIDController flywheelPid;
    private final SimpleMotorFeedforward flywheelFf;

    private double pivotSetpoint = 0;

    private boolean hasNote = false;
    private boolean isIntaking = false;
    private int intakeCounter = 0;
    private boolean isIntakingSource = false;
    private boolean flywheelSpinup = false;
    private boolean isShooting = false;
    private int abort = 0;
    private int outCounter = 25;



    public Shooter() {
        instance = this;
        inputs = new ShooterIOInputsAutoLogged();
        io = Robot.isSimulation() ? new ShooterIOSim(inputs) : new ShooterIOSparkMax(inputs);

        pivotPid = new PIDController(Constants.Shooter.Control.kp, Constants.Shooter.Control.ki,
                Constants.Shooter.Control.kd);
        pivotFf = new ArmFeedforward(Constants.Shooter.Control.ks, Constants.Shooter.Control.kg,
                Constants.Shooter.Control.kv, Constants.Shooter.Control.ka);

        flywheelPid = new PIDController(0.0015, 0, 0);
        flywheelFf = new SimpleMotorFeedforward(0, 0.0021);
    }
    /**
     * Initialize the subsystem
     */
    public static void init() {
        if (instance == null) new Shooter();
    }



    public void updateInputs() {
        io.updateInputs();
        Logger.processInputs("Shooter", inputs);
    }

    @Override
    public void periodic() {

        inputs.pivotPositionSetpoint = pivotSetpoint;

        if (inputs.beamBreak && !hasNote)
            hasNote = true;

        if (isIntaking && hasNote) {
            isIntaking = false;
            intakeCounter = 3;
        }

        if (isIntakingSource && hasNote) {
            isIntakingSource = false;
        }

        if (isShooting && inputs.beamBreak)
            outCounter = 25;
        else if (isShooting) {
            outCounter--;
            if (outCounter == 0) {
                isShooting = false;
                hasNote = false;
            }
        }


        if (isIntaking && !inputs.beamBreak) {
            io.setFeedVolts(Constants.Shooter.Voltages.feedUp);
        }
        else if (isShooting) {
            io.setFeedVolts(12);
        }
        else if (isIntakingSource) {
            io.setFeedVolts(-Constants.Shooter.Voltages.feedUp);
        }
        else if (intakeCounter > 0) {
            io.setFeedVolts(-12);
            intakeCounter--;
        }
        else {
            io.setFeedVolts(0);
        }

        if (isShooting || flywheelSpinup)
            io.setFlywheelVolts(flywheelPid.calculate(getFlywheelVelocityI(), 4000) +
                    flywheelFf.calculate(getFlywheelVelocityI()));
        else if (isIntakingSource)
            io.setFlywheelVolts(-3);
        else
            io.setFlywheelVolts(0);

        if (abort > 0) {
            io.setFeedVolts(-3);
            abort--;
        }



        io.setPivotVolts(pivotPid.calculate(inputs.pivotPosition, pivotSetpoint) +
                pivotFf.calculate(AngleUtil.degToRad(inputs.pivotPosition +
                        Constants.Shooter.Control.comAngleCompensation), AngleUtil.degToRad(inputs.pivotVelocity)));

        Logger.processInputs("Shooter", inputs);
    }



    /**
     * Sets the target position of the shooter pivot to {@value Constants.Shooter.Setpoints#tuck}
     */
    public static void setPivotPositionTuck() {
        instance.setPivotPositionI(Constants.Shooter.Setpoints.tuck);
    }
    /**
     * Sets the target position of the shooter pivot to {@value Constants.Shooter.Setpoints#hover}
     */
    public static void setPivotPositionHover() {
        instance.setPivotPositionI(Constants.Shooter.Setpoints.hover);
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
     * @param degrees The target position of the pivot in degrees from 0 to {@value Constants.Shooter.Setpoints#max}
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
        if (!hasNote)
            isIntaking = intaking;
        else
            isIntaking = false;
    }
    public static void setIntakingSource(boolean intaking) { instance.setIntakingSourceI(intaking); }
    private void setIntakingSourceI(boolean intaking) { isIntakingSource = intaking; }
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
        if (hasNote) {
            flywheelSpinup = false;
            isShooting = true;
        }
        else {
            flywheelSpinup = false;
            isShooting = false;
        }
    }

    public static void abortHandoff() {

    }
    private void abortHandoffI() {
        abort = 50;
        isShooting = false;
        flywheelSpinup = false;
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
