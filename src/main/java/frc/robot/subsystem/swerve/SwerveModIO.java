package frc.robot.subsystem.swerve;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

/**
 * The class representing the hardware inputs and outputs for the {@link SwerveMod} {@link Subsystem}
 */
public abstract class SwerveModIO {
    /**
     * A class containing all hardware / sensor inputs for the {@link SwerveMod} {@link Subsystem}
     */
    @AutoLog
    public static class SwerveModIOInputs {
        public double drivePositionDeg;
        public double driveVelocityDegSec;

        public double anglePositionDeg;
        public double anglePositionAbsoluteDeg;
        public double angleVelocityDegSec;
    }

    /**
     * A logged version of all hardware / sensor inputs for the {@link SwerveMod} {@link Subsystem}
     * as defined in {@link SwerveModIOInputs}
     */
    protected SwerveModIOInputsAutoLogged inputs = new SwerveModIOInputsAutoLogged();

    /**
     * Updates the hardware / sensor inputs for the {@link SwerveMod} {@link Subsystem}
     */
    public abstract void updateInputs();

    /**
     * Sets the voltage for the drive motor
     * @param volts The voltage to be applied
     */
    public abstract void setDriveVolts(double volts);

    /**
     * Sets the voltage for the angle motor
     * @param volts The voltage to be applied
     */
    public abstract void setAngleVolts(double volts);

    /**
     * Syncs the relative encoder on the angle motor to the absolute encoder on the module
     */
    public abstract void syncEncoders();

    /**
     * Sets the brake mode of the swerve module
     * @param brake Whether the module should be in brake mode
     */
    public abstract void setBrakeMode(boolean brake);
}
