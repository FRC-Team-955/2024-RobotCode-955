package frc.robot.command.handoff;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;

public class HandoffPosition extends Command {
    public HandoffPosition() {
        addRequirements(Intake.instance, Shooter.instance);
    }

    @Override
    public void initialize() {
        Intake.movePositionHandoff();
        Shooter.setPivotPositionLoad();
    }
}
