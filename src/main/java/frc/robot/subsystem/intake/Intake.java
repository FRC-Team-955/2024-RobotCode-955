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

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    private double targetPosition;
    private boolean reached = false;
    private boolean slam = false;
    private boolean slamOut = false;
    private final PIDController extendController;



    private Intake() {
        inputs = new IntakeIOInputsAutoLogged();
        io = Robot.isSimulation() ? new IntakeIOSim(inputs) : new IntakeIOSparkMax(inputs);
        extendController = new PIDController(0, 0, 0);
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
    public void slamPosition(double position, boolean out) {
        slam = true;
        slamOut = out;
        targetPosition = position;
    }

    /**
     * Moves the intake to the specified position
     * @param position The position for the intake to move to
     */
    public void movePosition(double position) {
        slam = false;
        reached = false;
        targetPosition = position;
        extendController.setSetpoint(position);
    }

    /**
     * Set the percentage speed of the intake motor (not the deploy motor)
     * @param percent The percentage speed (-1 to 1)
     */
    public void setIntakePercent(double percent) {
        io.setIntakeMotor(MathUtil.clamp(percent, -1, 1) * 12.0);
    }



    /**
     * Gets whether a note had been grabbed by the intake
     * @return Whether there is a note in the intake
     */
    public boolean noteCaptured() { return inputs.noteCaptured; }

    /**
     * Gets whether a note has been secured by the intake
     * @return Whether there is a note fully in the intake
     */
    public boolean noteSecured() { return inputs.noteSecured; }
}
