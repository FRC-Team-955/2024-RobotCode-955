package frc.robot.subsystem.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

/**
 * The class representing the hardware / sensor inputs for the {@link Intake} {@link Subsystem}
 */
public abstract class IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double position;
        public double positionSetpoint;
        public double velocity;
        public double intakePercent;
        public double intakeVelocity;
        public double intakeAmpDraw;
        public double ultrasonicRange;
        public double voltsAppliedDeploy;
        public double voltsAppliedIntake;
        public boolean brakeDeploy;
    }

    /**
     * A logged version of all hardware / sensor inputs for the {@link Intake} {@link Subsystem}
     * as defined in {@link IntakeIOInputs}
     */
    protected IntakeIOInputsAutoLogged inputs;

    /**
     * Updates the hardware / sensor inputs for the {@link Intake} {@link Subsystem}
     */
    public abstract void updateInputs();

    /**
     * Sets the voltage for the deploy motor
     * @param volts The voltage to be applied
     */
    public abstract void setDeployMotor(double volts);

    /**
     * Sets the voltage for the intake motor
     * @param volts The voltage to be applied
     */
    public abstract void setIntakeMotor(double volts);

    /**
     * Sets the brake mode of the intake
     * @param brake Whether the deploy motor should be in brake mode
     */
    public abstract void setDeployBrake(boolean brake);
}
