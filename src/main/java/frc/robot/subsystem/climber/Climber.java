package frc.robot.subsystem.climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystem.climber.ClimberIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The Climber {@link Subsystem} for completing the endgame stage chain climb
 */
public class Climber extends SubsystemBase {

    public static Climber instance;

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;

    public static boolean leftBrake = false;
    public static boolean rightBrake = false;

    private Climber() {
        instance = this;
        inputs = new ClimberIOInputsAutoLogged();
        io = (Robot.isSimulation()) ? new ClimberIOSim(inputs) : new ClimberIOSparkMax(inputs);
    }

    /**
     * Initialize the subsystem
     */
    public static void init() {
        if (instance == null) new Climber();
    }



    public void updateInputs() {
        io.updateInputs();
//        Logger.processInputs("Climber", inputs);
    }

    @Override
    public void periodic() {
//        if (io.)
    }

    public static void setVoltage(double volts) {
        instance.setVoltageI(volts);
    }
    private void setVoltageI(double volts) {
        if (DriverStation.isTest()) {
            io.setLeftVolts(volts);
            io.setRightVolts(volts);
            return;
        }

        if (inputs.extensionPositionLeft < 1 && volts < 0)
            io.setLeftVolts(0);
        else
            io.setLeftVolts(volts);
        if (inputs.extensionPositionRight < 1 && volts < 0)
            io.setRightVolts(0);
        else
            io.setRightVolts(volts);
    }
    public static void setLeftBrake(boolean brake) {
        instance.setLeftBrakeI(brake);
    }
    private void setLeftBrakeI(boolean brake) { io.setLeftBrake(brake); }

    /**
     * Sets the brake mode of the right climber
     * @param brake Whether the climber should be in brake mode
     */
    public static void setRightBrake(boolean brake) {
        instance.setRightBrakeI(brake);
    }
    private void setRightBrakeI(boolean brake) {
        io.setRightBrake(brake);
    }
}
