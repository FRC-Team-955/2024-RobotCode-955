package frc.robot.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

/**
 * Dashboard number that:
 * - While in tuning mode, appears as a normal number
 * - While not in tuning mode, appears a skew value, meaning that instead of the original value being displayed, a skew amount will be displayed that is added to the original value
 */
public class SkewableDashboardNumber implements LoggedDashboardInput {
    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key, value);
        }

        public void fromLog(LogTable table) {
            value = table.get(key, value);
        }
    };

    private final String key;
    private final double defaultValue;
    private double value;
    private double skewAmount = 0.0;
    private boolean originalShown = false;

    public SkewableDashboardNumber(DashboardSubsystem subsystem, String key, double defaultValue) {
        this.key = subsystem.prefix() + "/" + key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;

        periodic();
        Logger.registerDashboardInput(this);
    }

    public double get() {
        return value + skewAmount;
    }

    public void periodic() {
        System.out.println("TODO: FIX THIS - SKEW AMOUNT NEEDS TO BE PROPERLY HANDLED");
        if (RobotState.Dashboard.tuningMode.get() && !originalShown) {
            // Switch to original mode
            SmartDashboard.putNumber(key, value + skewAmount);
            skewAmount = 0.0;
            originalShown = true;
        } else if (!RobotState.Dashboard.tuningMode.get() && originalShown) {
            // Switch to skew mode
            skewAmount = value - defaultValue;
            SmartDashboard.putNumber(key, skewAmount);
            originalShown = false;
        }

        if (!Logger.hasReplaySource()) {
            value = SmartDashboard.getNumber(key, value);
        }
        Logger.processInputs(prefix, inputs);
    }
}
