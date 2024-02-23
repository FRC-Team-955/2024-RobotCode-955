package frc.robot.subsystem.shooterV1;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

/**
 * The class representing the hardware inputs and outputs for the {@link ShooterV1} {@link Subsystem}
 */
public abstract class ShooterIOV1 {
    /**
     * A class containing all hardware / sensor inputs for the {@link ShooterV1} {@link Subsystem}
     */
    @AutoLog
    public static class ShooterIOInputsV1 {
        public double pivotPosition;
        public double pivotPositionSetpoint;
        public double pivotVelocity;
        public double feedPosition;
        public double feedVelocity;
        public double flywheelPositionLeft;
        public double flywheelVelocityLeft;
        public double flywheelPositionRight;
        public double flywheelVelocityRight;
        public double ultrasonicRange;
        public double voltsAppliedPivot;
        public double voltsAppliedFeed;
        public double voltsAppliedLeft;
        public double voltsAppliedRight;
        public boolean brakeFlywheel;
    }

    /**
     * A logged version of all hardware / sensor inputs for the {@link ShooterV1} {@link Subsystem}
     * as defined in {@link ShooterIOInputsV1}
     */
    protected ShooterIOInputsV1AutoLogged inputs;

    /**
     * Updates the hardware / sensor inputs for the {@link ShooterV1} {@link Subsystem}
     */
    public abstract void updateInputs();

    /**
     * Sets the voltage for the pivot motor
     * @param volts The voltage to be applied
     */
    public abstract void setPivotVolts(double volts);
    /**
     * Sets the voltage for the feed motor
     * @param volts The voltage to be applied
     */
    public abstract void setFeedVolts(double volts);
    /**
     * Sets the voltage for the left flywheel motor
     * @param volts The voltage to be applied
     */
    public abstract void setFlywheelLeftVolts(double volts);
    /**
     * Sets the voltage for the right flywheel motor
     * @param volts The voltage to be applied
     */
    public abstract void setFlywheelRightVolts(double volts);

    /**
     * Sets the brake mode on the flywheel motors
     * @param brake Whether the flywheels should be in brake mode
     */
    public abstract void setFlywheelBrake(boolean brake);
}
