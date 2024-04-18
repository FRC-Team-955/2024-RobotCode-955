package frc.lib.util.absoluteencoder;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

public abstract class AbsoluteEncoder {
    public abstract Measure<Angle> getAbsolutePosition();

    public abstract boolean isConnected();
}
