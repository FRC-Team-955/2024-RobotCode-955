package frc.robot.command.handoff;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;

public class HandoffSubwoofer extends Command {

    public HandoffSubwoofer() {
        addRequirements(Intake.instance, Shooter.instance);
    }

    @Override
    public void initialize() {
        Intake.movePosition(Constants.Intake.Setpoints.handoff);
        Shooter.setPivotPosition(Constants.Shooter.Setpoints.load);
    }

    @Override
    public void execute() {
        if (Intake.atSetpoint() && Shooter.atPivotSetpoint()) {
            Shooter.setNotePositionSafe(Constants.Shooter.ContactRanges.flywheelStart);
            Intake.setIntakePercentHandoff();
        }
    }

    @Override
    public boolean isFinished() {
        return Shooter.atNoteSetpoint();
    }
}
