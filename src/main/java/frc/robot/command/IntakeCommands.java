package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;

public class IntakeCommands {
    /** Moves intake deploy position to the specified setpoint. */
    public static Command deployToSetpoint(double setpoint) {
        return Intake.instance.startEnd(() -> Intake.setDeployPositionSetpoint(setpoint), () -> {})
                .until(Intake::atDeploySetpoint);
    }

    /** Sets intake speed to the specified setpoint. */
    public static Command speedToSetpoint(double setpoint) {
        return Intake.instance.startEnd(() -> Intake.setIntakeVelocitySetpoint(setpoint), () -> {})
                .until(Intake::atIntakeSetpoint);
    }
}
