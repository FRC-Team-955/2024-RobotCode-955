package frc.robot.command.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;

/** Moves intake deploy position to the specified setpoint. */
public class IntakeDeployToSetpoint extends Command  {
private final double setpoint;

public IntakeDeployToSetpoint(double setpoint) {
    this.setpoint = setpoint;
    addRequirements(Intake.instance);
}

    @Override
    public void initialize() {
        Intake.setDeployPositionSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return Intake.atDeploySetpoint();
    }
}
