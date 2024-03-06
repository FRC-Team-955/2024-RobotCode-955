package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.shooter.Shooter;

import java.lang.constant.Constable;
import java.util.function.BooleanSupplier;

public class ShooterIntakeSource extends Command {

    private final BooleanSupplier r;

    public ShooterIntakeSource(BooleanSupplier control) {
        r = control;
        addRequirements(Shooter.instance);
    }

    @Override
    public void initialize() {
        Shooter.setIntakingSource(true);
        Shooter.setPivotPosition(Constants.Shooter.Setpoints.source);
    }

    @Override
    public boolean isFinished() {
        return Shooter.hasNote() || !r.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setPivotPositionTuck();
    }

    @Override
    public Command.InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
