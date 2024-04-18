package frc.lib.util.absoluteencoder;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import static edu.wpi.first.units.Units.Rotations;

public class AbsoluteEncoderREVThroughBore extends AbsoluteEncoder {
    private final DutyCycleEncoder encoder;

    public AbsoluteEncoderREVThroughBore(int pwmID) {
        encoder = new DutyCycleEncoder(pwmID);
    }

    @Override
    public Measure<Angle> getAbsolutePosition() {
        return Rotations.of(encoder.getAbsolutePosition());
    }

    @Override
    public boolean isConnected() {
        return encoder.isConnected();
    }
}
