package frc.robot.subsystem.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The class representing the hardware inputs and outputs for the {@link Shooter} {@link Subsystem}
 */
public abstract class ShooterIO {
    protected ShooterIOInputs inputs;

    /**
     * Updates the hardware / sensor inputs for the {@link Shooter} {@link Subsystem}
     */
    public abstract void updateSensors();
    public abstract void updateApplications();

    public abstract void updatePivotController(double setpointDegrees, double feedforwardVolts);
    public abstract void updateFeedController(double setpointMetersPerSecond, double feedforwardVolts);
    public abstract void updateFlywheelController(double setpointMetersPerSecond,
                                                  double feedforwardVoltsTop, double feedforwardVoltsBottom);

    public abstract void zeroPivotRelative();
    public abstract void zeroPivotAbsolute();
}
