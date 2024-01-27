package frc.robot.subsystem.intake;

import org.littletonrobotics.junction.AutoLog;

public abstract class IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double position;
        public boolean noteDetected;
    }

    public abstract void updateInputs();

    public abstract void setDeployMotor(double volts);

    public abstract void setIntakeMotor(double volts);

    public abstract void setDeployBrake(boolean brake);
}
