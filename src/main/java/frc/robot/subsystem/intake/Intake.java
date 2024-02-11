package frc.robot.subsystem.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.conversion.AngleUtil;
import frc.robot.utility.information.MatchUtil;

/**
 * The Intake {@link Subsystem} for collecting notes from the ground
 */
public class Intake extends SubsystemBase {

    public static Intake instance;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    private double targetPosition;
    private boolean reached = false;
    private boolean slam = false;
    private boolean slamOut = false;
    private final PIDController extendPid;
    private final ArmFeedforward extendFf;



    private Intake() {
        instance = this;
        inputs = new IntakeIOInputsAutoLogged();
        io = Robot.isSimulation() ? new IntakeIOSim(inputs) : new IntakeIOSparkMax(inputs);
        extendPid = new PIDController(Constants.Intake.Control.kp, Constants.Intake.Control.ki,
                Constants.Intake.Control.kd);
        extendFf = new ArmFeedforward(Constants.Intake.Control.ks, Constants.Intake.Control.kg,
                Constants.Intake.Control.kv, Constants.Intake.Control.ka);
    }

    /**
     * Initialize the subsystem
     */
    public static void init() {
        if (instance == null) new Intake();
    }



    public void updateInputs() {
        io.updateInputs();
    }

    @Override
    public void periodic() {
        if (slam) {
            if (slamOut && inputs.position < targetPosition) {
                io.setDeployMotor(12.0);
            }
            else if (!slamOut && inputs.position > targetPosition) {
                io.setDeployMotor(-12.0);
            }
        }
        else {
            if (!reached) {
                io.setDeployMotor(extendPid.calculate(inputs.position) +
                        extendFf.calculate(AngleUtil.degToRad(inputs.position), AngleUtil.degToRad(inputs.velocity)));
                if (Math.abs(inputs.position - targetPosition) < 5) reached = true;
            }
            else
                io.setDeployMotor(0);
        }

        if (MatchUtil.isPrematch()) {
            io.setDeployBrake(false);
        }
        else {
            io.setDeployBrake(inputs.position < Constants.Intake.extrusionThreshold);
        }
    }



    /**
     * Slams the intake to {@value Constants.Intake.Setpoints#handoff} as fast as possible
     */
    public static void slamPositionHandoff() {
        instance.slamPositionI(Constants.Intake.Setpoints.handoff, false);
    }
    /**
     * Slams the intake to {@value Constants.Intake.Setpoints#intake} as fast as possible
     */
    public static void slamPositionIntake() {
        instance.slamPositionI(Constants.Intake.Setpoints.intake, true);
    }
    /**
     * Slams the intake to the specified position as fast as possible
     * @param position The position for the intake to slam to
     * @param out Whether the intake is intended to slam outwards
     */
    public static void slamPosition(double position, boolean out) {
        instance.slamPositionI(position, out);
    }
    private void slamPositionI(double position, boolean out) {
        slam = true;
        slamOut = out;
        targetPosition = position;
    }

    /**
     * Moves the intake to {@value Constants.Intake.Setpoints#handoff}
     */
    public static void movePositionHandoff() {
        instance.movePositionI(Constants.Intake.Setpoints.handoff);
    }
    /**
     * Moves the intake to {@value Constants.Intake.Setpoints#hover}
     */
    public static void movePositionHover() {
        instance.movePositionI(Constants.Intake.Setpoints.hover);
    }
    /**
     * Moves the intake to {@value Constants.Intake.Setpoints#intake}
     */
    public static void movePositionIntake() {
        instance.movePositionI(Constants.Intake.Setpoints.intake);
    }
    /**
     * Moves the intake to the specified position
     * @param position The position for the intake to move to
     */
    public static void movePosition(double position) {
        instance.movePositionI(position);
    }
    private void movePositionI(double position) {
        slam = false;
        reached = false;
        targetPosition = position;
        extendPid.setSetpoint(position);
    }




    /**
     * Sets the percentage speed of the intake motor to {@value Constants.Intake.Percents#handoff}
     */
    public static void setIntakePercentHandoff() {
        instance.movePositionI(Constants.Intake.Percents.handoff);
    }
    /**
     * Sets the percentage speed of the intake motor to {@value Constants.Intake.Percents#hold}
     */
    public static void setIntakePercentHold() {
        instance.movePositionI(Constants.Intake.Percents.hold);
    }
    /**
     * Sets the percentage speed of the intake motor to {@value Constants.Intake.Percents#intake}
     */
    public static void setIntakePercentIntake() {
        instance.movePositionI(Constants.Intake.Percents.intake);
    }
    /**
     * Sets the percentage speed of the intake motor
     * @param percent The percentage speed (-1 to 1)
     */
    public static void setIntakePercent(double percent) {
        instance.movePositionI(percent);
    }
    private void setIntakePercentI(double percent) {
        io.setIntakeMotor(MathUtil.clamp(percent, -1, 1) * 12.0);
    }


    /**
     * Gets the current position of the intake
     * @return The current position of the intake deploy pivot
     */
    public static double getPosition() { return instance.getPositionI(); }
    private double getPositionI() { return inputs.position; }



    /**
     * Gets whether the deploy pivot is at the angular position setpoint
     * @return Whether the right flywheel is within {@value Constants.Intake#tolerance} degrees from the setpoint
     */
    public static boolean atSetpoint() {
        return instance.atSetpointI(Constants.Intake.tolerance);
    }
    /**
     * Gets whether the deploy pivot is at the angular position setpoint within a given tolerance
     * @param tolerance The setpoint error tolerance in degrees
     * @return Whether the right flywheel is within the given tolerance from the setpoint
     */
    public static boolean atSetpoint(double tolerance) {
        return instance.atSetpointI(tolerance);
    }
    private boolean atSetpointI(double tolerance) {
        return Math.abs(targetPosition - inputs.position) <= tolerance;
    }

    /**
     * Gets whether a note had been grabbed by the intake
     * @return Whether there is a note in the intake
     */
    public static boolean noteCaptured() {
        return instance.noteCapturedI();
    }
    private boolean noteCapturedI() { return inputs.noteCaptured; }

    /**
     * Gets whether a note has been secured by the intake
     * @return Whether there is a note fully in the intake
     */
    public static boolean noteSecured() {
        return instance.noteSecuredI();
    }
    private boolean noteSecuredI() { return inputs.noteSecured; }
}
