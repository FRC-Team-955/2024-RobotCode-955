package frc.lib.util.absoluteencoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteEncoderIOREVThroughBore extends AbsoluteEncoderIO {
    private final DutyCycleEncoder encoder;

    private double gearRatio;
    private double offsetRad;

    public AbsoluteEncoderIOREVThroughBore(int pwmID) {
        encoder = new DutyCycleEncoder(pwmID);
    }

    @Override
    public void updateInputs(AbsoluteEncoderIOInputs inputs) {
        inputs.isConnected = encoder.isConnected();
        inputs.positionRad = Units.rotationsToRadians(encoder.getAbsolutePosition()) / gearRatio - offsetRad;
    }

    @Override
    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }

    @Override
    public void setOffset(double offsetRad) {
        this.offsetRad = offsetRad;
    }
}
