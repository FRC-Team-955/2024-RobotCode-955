package frc.robot.subsystem.climber;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

/**
 * The class representing the hardware inputs and outputs for the {@link Climber} {@link Subsystem}
 */
public abstract class ClimberIO {
    /**
     * A class containing all hardware / sensor inputs for the {@link Climber} {@link Subsystem}
     */
    @AutoLog
    public static class ClimberIOInputs {
        public double extensionPositionLeft;
        public double extensionVelocityLeft;
        public double extensionPositionRight;
        public double extensionVelocityRight;
    }

    /**
     * A logged version of all hardware / sensor inputs for the {@link Climber} {@link Subsystem}
     * as defined in {@link ClimberIOInputs}
     */
    protected ClimberIOInputsAutoLogged inputs;

    /**
     * Updates the hardware / sensor inputs for the {@link Climber} {@link Subsystem}
     */
    public abstract void updateInputs();

    /**
     * Sets the voltage for the left climber motor
     * @param volts The voltage to be applied
     */
    public abstract void setLeftVolts(double volts);

    /**
     * Sets the voltage for the right climber motor
     * @param volts The voltage to be applied
     */
    public abstract void setRightVolts(double volts);

    /**
     * Sets the brake mode of the left climber
     * @param brake Whether the climber should be in brake mode
     */
    public abstract void setLeftBrake(boolean brake);

    /**
     * Sets the brake mode of the right climber
     * @param brake Whether the climber should be in brake mode
     */
    public abstract void setRightBrake(boolean brake);
}
