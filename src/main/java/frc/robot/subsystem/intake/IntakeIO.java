package frc.robot.subsystem.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The class representing the hardware / sensor inputs for the {@link Intake} {@link Subsystem}
 */
public abstract class IntakeIO {

    /**
     * A logged version of all hardware / sensor inputs for the {@link Intake} {@link Subsystem}
     * as defined in {@link IntakeIOInputs}
     */
    protected IntakeIOInputs inputs;

    /**
     * Updates the hardware / sensor inputs for the {@link Intake} {@link Subsystem}
     */
    public abstract void updateSensors();
    public abstract void updateApplications();

    public abstract void setDeployController(double setpointDegrees, double feedforwardVolts);
    public abstract void setIntakeController(double setpointMetersPerSecond, double feedforwardVolts);

    public abstract void setDeployVolts(double volts);
    public abstract void setIntakeVolts(double volts);

    public abstract void zeroDeployRelative();
    public abstract void zeroDeployAbsolute();
}
