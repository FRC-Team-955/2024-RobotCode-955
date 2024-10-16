package frc.robot.dashboard;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public class TuningDashboardAnglularVelocityRPM extends TuningDashboardNumber {
    public TuningDashboardAnglularVelocityRPM(DashboardSubsystem subsystem, String key, Measure<Velocity<Angle>> defaultValue) {
        super(subsystem, key + " (RPM)", defaultValue.in(RotationsPerSecond));
    }

    public Measure<Velocity<Angle>> get() {
        return RotationsPerSecond.of(getRaw());
    }
}
