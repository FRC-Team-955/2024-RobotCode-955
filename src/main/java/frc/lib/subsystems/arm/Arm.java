package frc.lib.subsystems.arm;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;

public final class Arm {
    private final SubsystemBase parent;
    private final String inputsName;
    public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final ArmIO io;

    private final ArmFeedforward ff;
    private Double setpointRad = null;
    private final Measure<Angle> initialPosition;

    /**
     * @param inputName Example: Intake/Pivot
     * @param gearRatio          >1 means a reduction, <1 means a upduction
     * @param initialPositionRad Initial position of the arm. 0 means parallel to the ground
     */
    public Arm(SubsystemBase parent, String inputName, ArmIO io, ArmFeedforward ff, PIDConstants pidConstants, double gearRatio, Measure<Angle> initialPosition) {
        this.parent = parent;
        this.inputsName = inputName;
        this.io = io;
        this.ff = ff;
        this.initialPosition = initialPosition;

        io.configurePID(pidConstants);
        io.setGearRatio(gearRatio);
        io.setPosition(initialPosition.in(Radians));
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/" + inputsName, inputs);

        if (RobotState.isEnabled() && setpointRad != null) {
            io.setSetpoint(setpointRad, ff.calculate(setpointRad, 0));
        }
    }

    public void setPercent(double percent) {
        io.setVoltage(percent * 12);
        setpointRad = null;
    }

    public void setSetpoint(Measure<Angle> setpoint) {
        this.setpointRad = setpoint.in(Radians);
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.positionRad - setpointRad) <= 5;
    }

    /**
     * Zeros to the initial position.
     */
    public void setPosition() {
        setPosition(initialPosition);
    }

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

    public Command setSetpointCommand(Measure<Angle> setpoint) {
        return parent.runOnce(() -> setSetpoint(setpoint));
    }

    public Command pivotTo(Measure<Angle> position) {
        return parent.startEnd(
                () -> setSetpoint(position),
                () -> {
                }
        ).until(this::atSetpoint);
    }

    /**
     * Zeros to the initial position.
     */
    public Command setPositionCommand() {
        return parent.runOnce(this::setPosition);
    }

    public Command setPositionCommand(Measure<Angle> position) {
        return parent.runOnce(() -> setPosition(position));
    }

    public Command setBreakModeCommand(boolean enabled) {
        return parent.runOnce(() -> setBreakMode(enabled));
    }
}
