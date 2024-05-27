package frc.lib.util.absoluteencoder;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.Radians;

public class AbsoluteEncoder {
    private final AbsoluteEncoderIOInputsAutoLogged inputs = new AbsoluteEncoderIOInputsAutoLogged();
    private final AbsoluteEncoderIO io;

    public AbsoluteEncoder(AbsoluteEncoderIO io, Measure<Angle> offset) {
        this.io = io;

        io.setOffset(offset.in(Radians));
    }

    /**
     * @param gearRatio >1 means a reduction, <1 means a upduction
     */
    public AbsoluteEncoder(
            AbsoluteEncoderIO io,
            Double gearRatio,
            Measure<Angle> offset
    ) {
        this.io = io;

        if (gearRatio != null) io.setGearRatio(gearRatio);
        io.setOffset(offset.in(Radians));
    }

    /**
     * Inputs will not be logged by this class. You must log the returned inputs yourself.
     */
    public AbsoluteEncoderIOInputsAutoLogged updateInputs() {
        io.updateInputs(inputs);
        return inputs;
    }

    public boolean isConnected() {
        return inputs.isConnected;
    }

    public Measure<Angle> getPosition() {
        return Radians.of(inputs.positionRad);
    }

    public void setGearRatio(double gearRatio) {
        io.setGearRatio(gearRatio);
    }
}
