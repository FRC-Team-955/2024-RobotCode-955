package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public class FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double leaderCurrentAmps = 0.0;
        public double followerCurrentAmps = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    public void updateInputs(FlywheelIOInputs inputs) {
    }

    /**
     * Run open loop at the specified voltage.
     */
    public void setVoltage(double volts) {
    }

    /**
     * Run closed loop at the specified velocity.
     */
    public void setVelocity(double velocityRadPerSec, double ffVolts) {
    }

    /**
     * Stop in open loop.
     */
    public void stop() {
    }

    /**
     * Set velocity PID constants.
     */
    public void configurePID(double kP, double kI, double kD) {
    }
}
