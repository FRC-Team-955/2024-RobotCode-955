package frc.robot.command.handoff;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;

public class HandoffFull extends Command {

    int state = 0;

    public HandoffFull() {
        addRequirements(Intake.instance, Shooter.instance);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return state == 0 ? InterruptionBehavior.kCancelSelf : InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public void initialize() {
        Intake.movePositionHandoff();
        Shooter.setPivotPositionLoad();
        Shooter.setFlywheelVelocityZero();
        Shooter.setFlywheelIndexing(true);
    }

    @Override
    public void execute() {
        switch (state) {
            default:
                if (Intake.atSetpoint() && Shooter.atPivotSetpoint()) {
                    Intake.setIntakePercent(Constants.Intake.Percents.handoff);
                    Shooter.setNotePosition(Constants.Shooter.ContactRanges.held);
                    state++;
                }
                break;
            case 2:
                if (Shooter.atNoteSetpoint()) {
                    Intake.movePosition(Constants.Intake.Setpoints.hover);
                    Shooter.setPivotPosition(Constants.Shooter.Setpoints.tuck);
                    state++;
                }
                break;
            case 3:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return Constants.Shooter.Setpoints.load - Shooter.getPivotPosition() +
                Intake.getPosition() - Constants.Intake.Setpoints.handoff < Constants.Handoff.contactRange;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setFlywheelIndexing(false);
        if (!interrupted)
            Shooter.setNotePositionSafe(Constants.Shooter.ContactRanges.flywheelStart);
    }
}
