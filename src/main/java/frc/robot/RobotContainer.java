package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.lib.util.CommandNintendoSwitchProController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController controller = Constants.Simulation.useNintendoSwitchProController ? new CommandNintendoSwitchProController(0) : new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardNumber flywheelSpeedInput =
            new LoggedDashboardNumber("Flywheel Speed", 1500.0);

    public RobotContainer() {
        switch (Constants.mode) {
            case REAL -> {
                Util.registerFieldsForAutoLogOutput(
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOSparkMax(0),
                                new ModuleIOSparkMax(1),
                                new ModuleIOSparkMax(2),
                                new ModuleIOSparkMax(3)
                        ),
                        new Flywheel(new FlywheelIOSparkMax())
                );
            }

            case SIM -> {
                Util.registerFieldsForAutoLogOutput(
                        new Drive(
                                new GyroIO(),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim()
                        ),
                        new Flywheel(new FlywheelIOSim())
                );
            }

            case REPLAY -> {
                Util.registerFieldsForAutoLogOutput(
                        new Drive(
                                new GyroIO(),
                                new ModuleIO(),
                                new ModuleIO(),
                                new ModuleIO(),
                                new ModuleIO()
                        ),
                        new Flywheel(new FlywheelIO())
                );
            }
        }

        // Set up auto routines
        NamedCommands.registerCommand(
                "Run Flywheel",
                Flywheel.get()
                        .startEnd(
                                () -> Flywheel.get().runVelocity(flywheelSpeedInput.get()), Flywheel.get()::stop)
                        .withTimeout(5.0)
        );
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                Drive.get().sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                Drive.get().sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                Drive.get().sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                Drive.get().sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );
        autoChooser.addOption(
                "Flywheel SysId (Quasistatic Forward)",
                Flywheel.get().sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Flywheel SysId (Quasistatic Reverse)",
                Flywheel.get().sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        autoChooser.addOption(
                "Flywheel SysId (Dynamic Forward)",
                Flywheel.get().sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Flywheel SysId (Dynamic Reverse)",
                Flywheel.get().sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        Drive.get().setDefaultCommand(
                Drive.get().joystickDrive(
                        controller::getLeftY,
                        controller::getLeftX,
                        () -> -controller.getRightX()
                )
        );

        controller.x().onTrue(Drive.get().stopWithXCommand());
        controller.rightBumper().onTrue(Drive.get().resetRotationCommand());
        controller
                .a()
                .whileTrue(
                        Flywheel.get()
                                .startEnd(
                                        () -> Flywheel.get().runVelocity(flywheelSpeedInput.get()),
                                        Flywheel.get()::stop));
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing.
        return autoChooser.get();
    }
}
