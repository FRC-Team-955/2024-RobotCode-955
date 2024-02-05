package frc.robot.subsystem.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.MatchUtil;

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
    private final PIDController extendController;



    private Intake() {
        instance = this;
        inputs = new IntakeIOInputsAutoLogged();
        io = Robot.isSimulation() ? new IntakeIOSim(inputs) : new IntakeIOSparkMax(inputs);
        extendController = new PIDController(0, 0, 0);
    }

    /**
     * Initialize the subsystem
     */
    public static void init() {
        if (instance == null) new Intake();
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
                io.setDeployMotor(extendController.calculate(inputs.position));
                if (Math.abs(inputs.position - targetPosition) < 5) reached = true;
            }
            else
                io.setDeployMotor(0);
        }

        if (MatchUtil.isPrematch()) {
            io.setDeployBrake(false);
        }
        else {
            io.setDeployBrake(inputs.position > Constants.Intake.extrusionThreshold);
        }
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
        extendController.setSetpoint(position);
    }

    /**
     * Set the percentage speed of the intake motor (not the deploy motor)
     * @param percent The percentage speed (-1 to 1)
     */
    public static void setIntakePercent(double percent) {
        instance.movePositionI(percent);
    }
    private void setIntakePercentI(double percent) {
        io.setIntakeMotor(MathUtil.clamp(percent, -1, 1) * 12.0);
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
