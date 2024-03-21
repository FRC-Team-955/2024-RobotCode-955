package frc.robot.utility.object;

public class ControlMode {
    public enum Tuning {
        Characterization,
        ControlLoop,
        Operation
    }
    public enum Actuator {
        Voltage,
        ControlLoop,
        MotionProfile
    }
}
