package frc.lib.subsystems.arm;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.RobotState;
import frc.lib.util.absoluteencoder.AbsoluteEncoder;
import frc.lib.util.absoluteencoder.AbsoluteEncoderIO;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;

public final class Arm {
    private static final double SETPOINT_TOLERANCE = Units.degreesToRadians(7);

    private final String ioName;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    public final ArmIO io;

    private final ArmFeedforward ff;
    private final AbsoluteEncoder absoluteEncoder;
    private Measure<Angle> initialPosition;
    public Double setpointRad;

    /**
     * @param ioName    Example: Intake/Pivot
     * @param gearRatio >1 means a reduction, <1 means a upduction
     */
    public Arm(
            String ioName,
            ArmIO io,
            ArmFeedforward ff,
            PIDConstants pidConstants,
            double gearRatio,
            AbsoluteEncoderIO absoluteEncoderIo,
            Measure<Angle> absoluteEncoderOffset
    ) {
        this.ioName = ioName;
        this.io = io;
        this.ff = ff;
        this.absoluteEncoder = new AbsoluteEncoder(absoluteEncoderIo, absoluteEncoderOffset);
        this.initialPosition = null;

        io.configurePID(pidConstants);
        io.setGearRatio(gearRatio);
    }

    /**
     * @param ioName          Example: Intake/Pivot
     * @param gearRatio       >1 means a reduction, <1 means a upduction
     * @param initialPosition Initial position of the arm. 0 means parallel to the ground.
     */
    public Arm(
            String ioName,
            ArmIO io,
            ArmFeedforward ff,
            PIDConstants pidConstants,
            double gearRatio,
            Measure<Angle> initialPosition
    ) {
        this.ioName = ioName;
        this.io = io;
        this.ff = ff;
        this.absoluteEncoder = null;
        this.initialPosition = initialPosition;

        io.configurePID(pidConstants);
        io.setGearRatio(gearRatio);
        io.setPosition(initialPosition.in(Radians));

        // Immediately start closed loop with the starting position as our setpoint
        setpointRad = initialPosition.in(Radians);
    }

    public void periodic() {
        if (absoluteEncoder != null) {
            Logger.processInputs("Inputs/" + ioName + "/AbsoluteEncoder", absoluteEncoder.updateInputs());

            // initialize initialPosition if we have an absolute encoder
            // must go before arm IO update so that the IO position is set
            if (initialPosition == null) {
                initialPosition = absoluteEncoder.getPosition();
                io.setPosition(initialPosition.in(Radians));

                // Immediately start closed loop with the initial position as our setpoint
                setpointRad = initialPosition.in(Radians);
            }
        }

        io.updateInputs(inputs);
        Logger.processInputs("Inputs/" + ioName, inputs);

        Logger.recordOutput(ioName + "/ClosedLoop", setpointRad != null);
        if (setpointRad != null) {
            Logger.recordOutput(ioName + "/Setpoint", setpointRad);

            if (RobotState.isEnabled()) {
                var ffVolts = ff.calculate(setpointRad, 0);
                Logger.recordOutput(ioName + "/FFVolts", ffVolts);
                io.setSetpoint(setpointRad, ffVolts);
            }
        }
    }

    public Measure<Angle> getPosition() {
        return Radians.of(inputs.positionRad);
    }

    public void setPercent(double percent) {
        io.setVoltage(percent * 12);
        setpointRad = null;
    }

    /**
     * 0 means parallel to the ground
     */
    public void setSetpoint(Measure<Angle> setpoint) {
        setpointRad = setpoint.in(Radians);
    }

    /**
     * Note: the tolerance is currently 5 degrees. If you need more
     * precision edit this class to take the tolerance as a parameter.
     */
    public boolean atSetpoint() {
        return Math.abs(inputs.positionRad - setpointRad) <= SETPOINT_TOLERANCE;
    }

    public void stop() {
        io.setVoltage(0);
        setpointRad = null;
    }

    /**
     * Tells the encoder the current position is the initial position.
     */
    public void setPosition() {
        setPosition(initialPosition);
    }

    /**
     * Tells the encoder the current position is the one specified.
     */
    public void setPosition(Measure<Angle> position) {
        io.setPosition(position.in(Radians));
    }

    public void setBreakMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }
}
