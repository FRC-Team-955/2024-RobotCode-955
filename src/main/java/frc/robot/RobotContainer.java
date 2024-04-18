package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystems.arm.ArmIO;
import frc.lib.subsystems.arm.ArmIOSim;
import frc.lib.subsystems.wheel.WheelIO;
import frc.lib.subsystems.wheel.WheelIOSim;
import frc.lib.util.CommandNintendoSwitchProController;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static edu.wpi.first.units.Units.RPM;

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
                        )
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
                        new Intake(new IntakeIO(), new ArmIOSim(DCMotor.getNEO(1), 0.3), new WheelIOSim(DCMotor.getNEO(1)))
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
                        new Intake(new IntakeIO(), new ArmIO(), new WheelIO())
                );
            }
        }

        // Set up auto routines
//        NamedCommands.registerCommand("Run Flywheel", Flywheel.get().run());
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

        // Configure the button bindings
        configureButtonBindings();

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
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

        controller.a().onTrue(Intake.get().feed.reachSetpointCommand(RPM.of(360)));
        controller.b().onTrue(Intake.get().feed.reachSetpointCommand(RPM.of(-360)));
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing.
        return autoChooser.get();
    }
}
