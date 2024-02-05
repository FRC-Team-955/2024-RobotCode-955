package frc.robot.subsystem.shooter;

import org.littletonrobotics.junction.AutoLog;

public abstract class ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double pivotPosition;
        public double pivotVelocity;
        public double feedPosition;
        public double feedVelocity;
        public double flywheelPositionLeft;
        public double flywheelVelocityLeft;
        public double flywheelPositionRight;
        public double flywheelVelocityRight;
        public double ultrasonicRange;
    }

    protected ShooterIOInputsAutoLogged inputs;

    public abstract void updateInputs();
    public abstract void setPivotVolts(double volts);
    public abstract void setFeedVolts(double volts);
    public abstract void setFlywheelLeftVolts(double volts);
    public abstract void setFlywheelRightVolts(double volts);
}
