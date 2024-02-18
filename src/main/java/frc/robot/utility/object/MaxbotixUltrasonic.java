package frc.robot.utility.object;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class MaxbotixUltrasonic {
    private final AnalogInput input;

    public MaxbotixUltrasonic(int port) {
        input = new AnalogInput(port);
    }

    public double getRangeCentimeters() {
        return input.getValue() * (5 / RobotController.getVoltage5V()) * 0.125;
    }
    public double getRangeMeters() {
        return input.getValue() * (5 / RobotController.getVoltage5V()) * 0.0125;
    }
    public double getRangeInches() {
        return input.getValue() * (5 / RobotController.getVoltage5V()) * 0.049212625;
    }
}
