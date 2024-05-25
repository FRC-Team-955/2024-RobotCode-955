package frc.lib.subsystems.wheel;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * Generic flywheel or roller
 */
public class Wheel {
    private final SubsystemBase parent;
    private final String inputsName;
    private final WheelIOInputsAutoLogged inputs = new WheelIOInputsAutoLogged();
    private final WheelIO io;


    private final SimpleMotorFeedforward ff;
    private final Measure<Velocity<Angle>> setpointTolerance;
    private Double setpointRadPerSec = null;

    /**
     * @param inputName Example: Intake/Feed
     * @param gearRatio >1 means a reduction, <1 means a upduction
     */
    public Wheel(
            SubsystemBase parent,
            String inputName,
            WheelIO io,
            SimpleMotorFeedforward ff,
            PIDConstants pidConstants,
            double gearRatio,
            Measure<Velocity<Angle>> setpointTolerance
    ) {
        this.parent = parent;
        this.inputsName = inputName;
        this.io = io;
        this.ff = ff;
        this.setpointTolerance = setpointTolerance;

        io.configurePID(pidConstants);
        io.setGearRatio(gearRatio);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/" + inputsName, inputs);

        Logger.recordOutput(inputsName + "/ClosedLoop", setpointRadPerSec != null);
        if (setpointRadPerSec != null) {
            Logger.recordOutput(inputsName + "/Setpoint", setpointRadPerSec);

            if (RobotState.isEnabled()) {
                var ffVolts = ff.calculate(setpointRadPerSec, 0);
                Logger.recordOutput(inputsName + "/FFVolts", ffVolts);
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
        return Math.abs(inputs.velocityRadPerSec - setpointRadPerSec) <= setpointTolerance.in(RadiansPerSecond);
    }

    public void stop() {
        io.stop();
        setpointRadPerSec = null;
    }

    public void setBreakMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    public Command setPercentCommand(double percent) {
        return parent.startEnd(
                () -> setPercent(percent),
                io::stop
        );
    }

    public Command setSetpointCommand(Measure<Velocity<Angle>> setpoint) {
        return parent.runOnce(() -> setSetpoint(setpoint));
    }

    /**
     * The returned command ends once the setpoint is reached.
     */
    public Command reachSetpointCommand(Measure<Velocity<Angle>> setpoint) {
        return parent.startEnd(
                () -> setSetpoint(setpoint),
                () -> {
                }
        ).until(this::atSetpoint);
    }

    public Command stopCommand() {
        return parent.runOnce(this::stop);
    }

    public Command setBreakModeCommand(boolean enabled) {
        return parent.runOnce(() -> setBreakMode(enabled));
    }
}
