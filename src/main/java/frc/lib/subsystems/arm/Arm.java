package frc.lib.subsystems.arm;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;

public final class Arm {
    private static final double SETPOINT_TOLERANCE = Units.degreesToRadians(5);

    private final SubsystemBase parent;
    private final String inputsName;
    public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final ArmIO io;

    private final ArmFeedforward ff;
    private final Measure<Angle> initialPosition;
    private Double setpointRad = null;

    /**
     * @param inputName       Example: Intake/Pivot
     * @param gearRatio       >1 means a reduction, <1 means a upduction
     * @param initialPosition Initial position of the arm. 0 means parallel to the ground
     */
    public Arm(
            SubsystemBase parent,
            String inputName,
            ArmIO io,
            ArmFeedforward ff,
            PIDConstants pidConstants,
            double gearRatio,
            Measure<Angle> initialPosition
    ) {
        this.parent = parent;
        this.inputsName = inputName;
        this.io = io;
        this.ff = ff;
        this.initialPosition = initialPosition;

        io.configurePID(pidConstants);
        io.setGearRatio(gearRatio);
        io.setPosition(initialPosition.in(Radians));

        // Immediately start closed loop with the starting position as our setpoint
        setpointRad = initialPosition.in(Radians);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/" + inputsName, inputs);

        Logger.recordOutput(inputsName + "/ClosedLoop", setpointRad != null);
        if (setpointRad != null) {
            Logger.recordOutput(inputsName + "/Setpoint", setpointRad);

            if (RobotState.isEnabled()) {
                var ffVolts = ff.calculate(setpointRad, 0);
                Logger.recordOutput(inputsName + "/FFVolts", ffVolts);
                io.setSetpoint(setpointRad, ffVolts);
            }
        }
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

    public boolean atSetpoint() {
        return Math.abs(inputs.positionRad - setpointRad) <= SETPOINT_TOLERANCE;
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

    public Command setPercentCommand(double percent) {
        return parent.startEnd(
                () -> setPercent(percent),
                io::stop
        );
    }

    /**
     * 0 means parallel to the ground
     */
    public Command setSetpointCommand(Measure<Angle> setpoint) {
        return parent.runOnce(() -> setSetpoint(setpoint));
    }

    /**
     * The returned command ends once the setpoint is reached.
     * <p>
     * 0 means parallel to the ground
     */
    public Command reachSetpointCommand(Measure<Angle> setpoint) {
        return parent.startEnd(
                () -> setSetpoint(setpoint),
                () -> {
                }
        ).until(this::atSetpoint);
    }

    /**
     * Tells the encoder the current position is the initial position.
     */
    public Command setPositionCommand() {
        return parent.runOnce(this::setPosition);
    }

    /**
     * Tells the encoder the current position is the one specified.
     */
    public Command setPositionCommand(Measure<Angle> position) {
        return parent.runOnce(() -> setPosition(position));
    }

    public Command setBreakModeCommand(boolean enabled) {
        return parent.runOnce(() -> setBreakMode(enabled));
    }
}
