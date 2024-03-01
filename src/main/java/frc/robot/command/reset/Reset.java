package frc.robot.command.reset;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;

public class Reset extends Command {

    public Reset() {
        addRequirements(Intake.instance, Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setPivotPositionTuck();
        Intake.movePositionHover();
    }

    @Override
    public boolean isFinished() {
        return Shooter.atPivotSetpoint() && Intake.atSetpoint();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
