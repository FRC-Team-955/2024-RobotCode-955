package frc.lib.util.absoluteencoder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.Rotations;

public class AbsoluteEncoderCANcoder extends AbsoluteEncoder {
    private final CANcoder cancoder;

    private final StatusSignal<Double> absolutePosition;
    private boolean isConnected = true;

    public AbsoluteEncoderCANcoder(int canID) {
        cancoder = new CANcoder(canID);
        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        absolutePosition = cancoder.getAbsolutePosition();
    }

    /**
     * Warning: if you need this multiple times, call it once and store it in a variable.
     */
    @Override
    public Measure<Angle> getAbsolutePosition() {
        isConnected = absolutePosition.refresh().getStatus().isOK();
        return Rotations.of(absolutePosition.getValue());
    }

    @Override
    public boolean isConnected() {
        return isConnected;
    }
}
