package frc.robot.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class DashboardNumber implements LoggedDashboardInput {
    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key, value);
        }

        public void fromLog(LogTable table) {
            value = table.get(key, defaultValue);
        }
    };

    private final String key;
    private final double defaultValue;
    private double value;

    public DashboardNumber(DashboardSubsystem subsystem, String key, double defaultValue) {
        this.key = subsystem.prefix() + "/" + key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;

        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
        periodic();
        Logger.registerDashboardInput(this);
    }

    public double get() {
        return value;
    }

    public void periodic() {
        if (!Logger.hasReplaySource()) {
            value = SmartDashboard.getNumber(key, defaultValue);
        }
        Logger.processInputs(prefix, inputs);
    }
}
