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
}
