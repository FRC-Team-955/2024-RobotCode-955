package frc.lib.util.absoluteencoder;

import org.littletonrobotics.junction.AutoLog;

public class AbsoluteEncoderIO {
    @AutoLog
    public static class AbsoluteEncoderIOInputs {
        public boolean isConnected = false;
        public double positionRad = 0.0;
    }

    public void updateInputs(AbsoluteEncoderIOInputs inputs) {
    }

    public void setGearRatio(double gearRatio) {
    }

    public void setOffset(double offsetRad) {
    }
}