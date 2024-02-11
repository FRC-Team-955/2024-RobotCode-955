package frc.robot.command.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.subsystem.intake.Intake;

public class IntakeCommand {

    public static Command toSetpoint(final double setpoint) {
        return new FunctionalCommand(() -> {
                Intake.movePosition(setpoint);
            }, ()->{}, (b)->{}, Intake::atSetpoint, Intake.instance);
    }

    public static Command toIntakePosition() {
        return toSetpoint(Constants.Intake.Setpoints.intake);
    }

    public static Command toHoverPosition() {
        return toSetpoint(Constants.Intake.Setpoints.hover);
    }

    public static Command toHandoffPosition() {
        return toSetpoint(Constants.Intake.Setpoints.handoff);
    }
}
