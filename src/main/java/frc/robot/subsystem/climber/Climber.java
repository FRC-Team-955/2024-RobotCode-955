package frc.robot.subsystem.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climber extends SubsystemBase {

    private Climber() {
        inputs = new ClimberIOValuesAutoLogged();
        io = (Robot.isSimulation()) ? new ClimberIOSim(inputs) : new ClimberIOSparkMax(inputs);
    }

    private final ClimberIO io;
    private final ClimberIOValuesAutoLogged inputs;

    private double left;
    private double right;

    public void setTargetLeft(double position) {
        left = position;
    }

    public void setTargetRight(double position) {
        right = position;
    }

    public void setLeftBrake(boolean brake) { io.setLeftBrake(brake); }

    public void setRightBrake(boolean brake) {
        io.setRightBrake(brake);
    }


    @Override
    public void periodic() {
        if (inputs.extentionPositionLeft < left)
            io.setLeftVolts(Constants.Climber.extendVoltage);
        else if (inputs.extentionPositionLeft > left)
            io.setLeftVolts(Constants.Climber.climbVoltage);
        else
            io.setLeftVolts(0);

        if (inputs.extentionPositionRight < right)
            io.setRightVolts(Constants.Climber.extendVoltage);
        else if (inputs.extentionPositionRight > right)
            io.setRightVolts(Constants.Climber.climbVoltage);
        else
            io.setRightVolts(0);
    }
}
