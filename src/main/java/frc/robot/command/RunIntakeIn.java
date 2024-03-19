package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;

import java.util.function.BooleanSupplier;

public class RunIntakeIn extends Command {
    private final BooleanSupplier r;

    public RunIntakeIn(BooleanSupplier control) {
        addRequirements(Intake.instance);
        r = control;
    }

    @Override
    public void initialize() {
        Intake.setIntakePercent(0);
    }

    @Override
    public void execute() {
        Intake.movePositionHover();
        if (Intake.atDeploySetpoint()) {
            Intake.setIntakePercentIntake();
        }
    }

    @Override
    public boolean isFinished() {
        return !r.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        Intake.setIntakePercent(0);
    }

    @Override
    public Command.InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
