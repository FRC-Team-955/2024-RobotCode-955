package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    public void updateInputs(ModuleIOInputs inputs) {
    }

    /**
     * Run the drive motor at the specified voltage.
     */
    public void setDriveVoltage(double volts) {
    }

    /**
     * Run the turn motor at the specified voltage.
     */
    public void setTurnVoltage(double volts) {
    }

    /**
     * Enable or disable brake mode on the drive motor.
     */
    public void setDriveBrakeMode(boolean enable) {
    }

    /**
     * Enable or disable brake mode on the turn motor.
     */
    public void setTurnBrakeMode(boolean enable) {
    }
}