package frc.robot.command.defaults;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.information.Input;

public class Drive extends Command {
    public Drive() {
        addRequirements(Swerve.instance);
    }

    @Override
    public void execute() {
        Swerve.drivePercents(new Translation2d(Input.driveX(), Input.driveY()), Input.rotation(), true);
    }
}
