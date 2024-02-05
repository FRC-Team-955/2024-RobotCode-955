package frc.robot.subsystem.climber;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * The Climber {@link Subsystem} for completing the endgame stage chain climb
 */
public class Climber extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;

    private double left;
    private double right;



    private Climber() {
        inputs = new ClimberIOInputsAutoLogged();
        io = (Robot.isSimulation()) ? new ClimberIOSim(inputs) : new ClimberIOSparkMax(inputs);
    }



    @Override
    public void periodic() {
        if (inputs.extensionPositionLeft < left)
            io.setLeftVolts(Constants.Climber.extendVoltage);
        else if (inputs.extensionPositionLeft > left)
            io.setLeftVolts(Constants.Climber.climbVoltage);
        else
            io.setLeftVolts(0);

        if (inputs.extensionPositionRight < right)
            io.setRightVolts(Constants.Climber.extendVoltage);
        else if (inputs.extensionPositionRight > right)
            io.setRightVolts(Constants.Climber.climbVoltage);
        else
            io.setRightVolts(0);
    }



    /**
     * Sets the target extension of the left climber
     * @param position The target extension of the left climber in meters
     */
    public void setTargetLeft(double position) {
        left = position;
    }

    /**
     * Sets the target extension of the right climber
     * @param position The target extension of the right climber in meters
     */
    public void setTargetRight(double position) {
        right = position;
    }

    /**
     * Sets the brake mode of the left climber
     * @param brake Whether the climber should be in brake mode
     */
    public void setLeftBrake(boolean brake) { io.setLeftBrake(brake); }

    /**
     * Sets the brake mode of the right climber
     * @param brake Whether the climber should be in brake mode
     */
    public void setRightBrake(boolean brake) {
        io.setRightBrake(brake);
    }
}
