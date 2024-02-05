package frc.robot.subsystem.climber;

import org.littletonrobotics.junction.AutoLog;

public abstract class ClimberIO {
    @AutoLog
    public static class ClimberIOValues {
        public double extentionPositionLeft;
        public double extentionVelocityLeft;
        public double extentionPositionRight;
        public double extentionVelocityRight;
    }

    protected ClimberIOValuesAutoLogged inputs;

    public abstract void updateInputs();

    public abstract void setLeftVolts(double volts);
    public abstract void setRightVolts(double volts);
    public abstract void setLeftBrake(boolean brake);
    public abstract void setRightBrake(boolean brake);
}
