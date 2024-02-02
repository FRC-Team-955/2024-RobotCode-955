package frc.robot.subsystem.swerve;

import org.littletonrobotics.junction.AutoLog;

public abstract class SwerveModIO {
    @AutoLog
    public static class SwerveModIOInputs {
        public double drivePositionDeg;
        public double driveVelocityDegSec;

        public double anglePositionDeg;
        public double anglePositionAbsoluteDeg;
        public double angleVelocityDegSec;
    }

    protected SwerveModIOInputsAutoLogged inputs = new SwerveModIOInputsAutoLogged();

    public abstract void updateInputs();

    public abstract void setDriveVolts(double volts);
    public abstract void setAngleVolts(double volts);
    public abstract void syncEncoders();
    public abstract void setBrakeMode(boolean brake);
}
