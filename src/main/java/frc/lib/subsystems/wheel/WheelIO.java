package frc.lib.subsystems.wheel;

import org.littletonrobotics.junction.AutoLog;

public class WheelIO {
    @AutoLog
    public static class WheelIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public void updateInputs(WheelIOInputs inputs) {
    }
}
