package frc.robot.command.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;

/** Sets intake speed to the specified setpoint. */
public class IntakeSpeedToSetpoint extends Command {
    private final double setpoint;

    public IntakeSpeedToSetpoint(double setpoint) {
        this.setpoint = setpoint;
        addRequirements(Intake.instance);
    }

    @Override
    public void initialize() {
        Intake.setIntakeVelocitySetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return Intake.atIntakeSetpoint();
    }
}
