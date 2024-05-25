package frc.lib.util.absoluteencoder;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static edu.wpi.first.units.Units.Radians;

public class AbsoluteEncoder {
    private final String inputsName;
    private final AbsoluteEncoderIOInputsAutoLogged inputs = new AbsoluteEncoderIOInputsAutoLogged();
    private final AbsoluteEncoderIO io;

    /**
     * @param inputName Example: Intake/Pivot/AbsoluteEncoder
     * @param gearRatio >1 means a reduction, <1 means a upduction
     */
    public AbsoluteEncoder(
            String inputName,
            AbsoluteEncoderIO io,
            double gearRatio,
            Measure<Angle> offset
    ) {
        this.inputsName = inputName;
        this.io = io;

        io.setGearRatio(gearRatio);
        io.setOffset(offset.in(Radians));
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/" + inputsName, inputs);
    }

    public boolean isConnected() {
        return inputs.isConnected;
    }

    public Optional<Measure<Angle>> getPosition() {
        return Radians.of(inputs.positionRad);
    }
}
