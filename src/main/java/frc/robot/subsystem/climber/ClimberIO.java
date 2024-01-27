package frc.robot.subsystem.climber;

import org.littletonrobotics.junction.AutoLog;

public abstract class ClimberIO {
    @AutoLog
    public static class ClimberIOValues {
        public double extentionPositionLeft;
        public double extentionVelocityLeft;
        public double extentionPositionRight;
        public double extentionVelocityRight;

        public double appliedVoltsLeft;
        public double appliedVoltsRight;
    }

    public abstract void updateInputs(ClimberIOValues values);

    public abstract void moveLeft(double percent);
    public abstract void moveRight(double percent);
    public abstract void setLeftIdle(Climber.IdleMode mode);
    public abstract void setRightIdle(Climber.IdleMode mode);
}
