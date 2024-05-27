package frc.lib.subsystems.wheel;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotState;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * Generic flywheel or roller
 */
public class Wheel {
    private final String ioName;
    private final WheelIOInputsAutoLogged inputs = new WheelIOInputsAutoLogged();
    private final WheelIO io;

    private final SimpleMotorFeedforward ff;
    private final double setpointToleranceRadPerSec;
    private Double setpointRadPerSec = null;

    /**
     * @param ioName    Example: Intake/Feed
     * @param gearRatio >1 means a reduction, <1 means a upduction
     */
    public Wheel(
            String ioName,
            WheelIO io,
            SimpleMotorFeedforward ff,
            PIDConstants pidConstants,
            double gearRatio,
            Measure<Velocity<Angle>> setpointTolerance
    ) {
        this.ioName = ioName;
        this.io = io;
        this.ff = ff;
        this.setpointToleranceRadPerSec = setpointTolerance.in(RadiansPerSecond);

        io.configurePID(pidConstants);
        io.setGearRatio(gearRatio);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/" + ioName, inputs);

        Logger.recordOutput(ioName + "/ClosedLoop", setpointRadPerSec != null);
        if (setpointRadPerSec != null) {
            Logger.recordOutput(ioName + "/Setpoint", setpointRadPerSec);

            if (RobotState.isEnabled()) {
                var ffVolts = ff.calculate(setpointRadPerSec, 0);
                Logger.recordOutput(ioName + "/FFVolts", ffVolts);
                io.setSetpoint(setpointRadPerSec, ffVolts);
            }
        }
    }

    public void setPercent(double percent) {
        io.setVoltage(percent * 12);
        setpointRadPerSec = null;
    }

    /**
     * 0 means parallel to the ground
     */
    public void setSetpoint(Measure<Velocity<Angle>> setpoint) {
        setpointRadPerSec = setpoint.in(RadiansPerSecond);
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.velocityRadPerSec - setpointRadPerSec) <= setpointToleranceRadPerSec;
    }

    public void stop() {
        io.setVoltage(0);
        setpointRadPerSec = null;
    }

    public void setBreakMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }
}
