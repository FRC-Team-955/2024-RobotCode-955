package frc.lib.util.absoluteencoder;

public class AbsoluteEncoderIOSim extends AbsoluteEncoderIO {
    private final double positionRad = Math.random() * 2.0 * Math.PI;

    @Override
    public void updateInputs(AbsoluteEncoderIOInputs inputs) {
        inputs.isConnected = true;
        inputs.positionRad = positionRad;
    }
}
