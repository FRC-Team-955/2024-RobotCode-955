package frc.robot.subsystem.swerve;

import com.revrobotics.CANSparkBase;
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

    private SwerveModIOInputsAutoLogged inputs;

    public abstract void setDriveVolts(double volts);
    public abstract void setAngleVolts(double volts);
    public abstract void syncEncoders();
    public abstract void setIdleMode(CANSparkBase.IdleMode mode);
}
