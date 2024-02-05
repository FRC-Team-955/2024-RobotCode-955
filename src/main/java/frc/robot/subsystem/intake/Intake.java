package frc.robot.subsystem.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.MatchUtil;

public class Intake extends SubsystemBase {

    private Intake() {
        inputs = new IntakeIOInputsAutoLogged();
        io = Robot.isSimulation() ? new IntakeIOSim(inputs) : new IntakeIOSparkMax(inputs);
        extendController = new PIDController(0, 0, 0);
    }

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    private double targetPosition;
    private boolean reached = false;
    private boolean slam = false;
    private boolean slamOut = false;
    private final PIDController extendController;

    public void slamPosition(double position, boolean out) {
        slam = true;
        slamOut = out;
        targetPosition = position;
    }

    public void movePosition(double position) {
        slam = false;
        reached = false;
        targetPosition = position;
        extendController.setSetpoint(position);
    }

    public void setIntakePercent(double percent) {
        io.setIntakeMotor(MathUtil.clamp(percent, -1, 1) * 12.0);
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

    public boolean noteCaptured() { return inputs.noteCaptured; }
    public boolean noteSecured() { return inputs.noteSecured; }
}
