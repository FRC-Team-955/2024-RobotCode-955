package frc.robot.subsystem.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Climber extends SubsystemBase {

    public static Climber instance = new Climber();

    private Climber() {
        inputs = new ClimberIOValuesAutoLogged();
        io = (Robot.isSimulation()) ? new ClimberIOSim(inputs) : new ClimberIOSparkMax(inputs);
    }

    public enum IdleMode {
        Brake,
        Free
    }

    private ClimberIO io;
    private ClimberIOValuesAutoLogged inputs;

    private double left;
    private double right;

    private PIDController pidLeft;
    private PIDController pidRight;

    public void setTargetLeft(double percent) {
        left = percent;
    }

    public void setTargetRight(double percent) {
        right = percent;
    }

    public void setModeLeft(IdleMode mode) {
        io.setLeftIdle(mode);
    }

    public void setModeRight(IdleMode mode) {
        io.setRightIdle(mode);
    }


    @Override
    public void periodic() {

    }
}
