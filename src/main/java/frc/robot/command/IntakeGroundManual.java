package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;

import java.util.function.BooleanSupplier;

public class IntakeGroundManual extends Command {

    private final BooleanSupplier r;

    public IntakeGroundManual(BooleanSupplier control) {
        r = control;
    }

    @Override
    public void initialize() {
        Intake.movePositionIntake();
        Intake.setIntakePercent(1);
    }

    @Override
    public boolean isFinished() {
        return !r.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
//        System.out.println(interrupted);
        Intake.movePositionHandoff();
        Intake.setIntakePercent(0);
    }
}
