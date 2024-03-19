package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystem.intake.Intake;

import java.util.function.BooleanSupplier;

public class IntakeGroundManual extends Command {

    private final BooleanSupplier r;

    public IntakeGroundManual(BooleanSupplier control) {
        addRequirements(Intake.instance);
        r = control;
    }

    @Override
    public void initialize() {
        Intake.setIntakePercent(1);
    }

    @Override
    public void execute() {
        Intake.movePositionIntake();
        if (Intake.atDeploySetpoint() && Intake.hasNote())
            RobotContainer.instance.rumbleControllers(true);
    }

    @Override
    public boolean isFinished() {
        return (!r.getAsBoolean() || Intake.hasNote()) && Intake.atDeploySetpoint();
    }

    @Override
    public void end(boolean interrupted) {
//        System.out.println(interrupted);
        RobotContainer.instance.rumbleControllers(false);
        Intake.movePositionHover();
        Intake.setIntakePercent(0);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
