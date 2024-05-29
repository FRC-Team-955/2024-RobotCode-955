package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystems.arm.ArmIO;
import frc.lib.subsystems.arm.ArmIOSim;
import frc.lib.subsystems.arm.ArmIOSparkMax;
import frc.lib.subsystems.wheel.WheelIO;
import frc.lib.subsystems.wheel.WheelIOSim;
import frc.lib.subsystems.wheel.WheelIOSparkMax;
import frc.lib.util.CommandNintendoSwitchProController;
import frc.lib.util.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.util.absoluteencoder.AbsoluteEncoderIOREVThroughBore;
import frc.lib.util.absoluteencoder.AbsoluteEncoderIOSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController driverController = Constants.Simulation.useNintendoSwitchProController ? new CommandNintendoSwitchProController(0) : new CommandXboxController(0);
    private final CommandXboxController operatorController = Constants.Simulation.useNintendoSwitchProController ? new CommandNintendoSwitchProController(1) : new CommandXboxController(1);

    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.mode) {
            case REAL -> {
                Util.registerFieldsForAutoLogOutput(
                        new Drive(
                                new GyroIOPigeon2(10),
                                new ModuleIOSparkMax(0),
                                new ModuleIOSparkMax(1),
                                new ModuleIOSparkMax(2),
                                new ModuleIOSparkMax(3)
                        ),
                        new Intake(
                                new IntakeIO(),
                                new ArmIOSparkMax(3),
                                new AbsoluteEncoderIO(),//REVThroughBore(0),
                                new WheelIOSparkMax(16, false, true)
                        ),
                        new Shooter(
                                new ShooterIOReal(6),
                                new ArmIOSparkMax(7),
                                new AbsoluteEncoderIOREVThroughBore(0),
                                new WheelIOSparkMax(9, false, false),
                                new WheelIOSparkMax(10, false, false),
                                new WheelIOSparkMax(8, false, false)
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
                        new Intake(
                                new IntakeIO(),
                                new ArmIOSim(DCMotor.getNEO(1), 0.3, 0.045),
                                new AbsoluteEncoderIOSim(),
                                new WheelIOSim(DCMotor.getNEO(1))
                        ),
                        new Shooter(
                                new ShooterIOSim(),
                                new ArmIOSim(DCMotor.getNEO(1), 0.4, 0.083),
                                new AbsoluteEncoderIOSim(),
                                new WheelIOSim(DCMotor.getNEO(1)),
                                new WheelIOSim(DCMotor.getNEO(1)),
                                new WheelIOSim(DCMotor.getNEO(1))
                        )
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
                        new Intake(new IntakeIO(), new ArmIO(), new AbsoluteEncoderIO(), new WheelIO()),
                        new Shooter(new ShooterIO(), new ArmIO(), new AbsoluteEncoderIO(), new WheelIO(), new WheelIO(), new WheelIO())
                );
            }
        }

        // Set up auto routines
//        NamedCommands.registerCommand("Run Flywheel", Flywheel.get().run());
        autoChooser = new LoggedDashboardChooser<>("Auto Choices"); //AutoBuilder.buildAutoChooser());

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

        setDefaultCommands();
        configureButtonBindings();

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    }

    private void setDefaultCommands() {
        Drive.get().setDefaultCommand(
                Drive.get().joystickDrive(
                        () -> {
                            var override =
                                    Math.abs(operatorController.getLeftX()) > 0.1 ||
                                            Math.abs(operatorController.getLeftY()) > 0.1 ||
                                            Math.abs(operatorController.getRightX()) > 0.1;
                            return override ? operatorController.getLeftY() : driverController.getLeftY();
                        },
                        () -> {
                            var override =
                                    Math.abs(operatorController.getLeftX()) > 0.1 ||
                                            Math.abs(operatorController.getLeftY()) > 0.1 ||
                                            Math.abs(operatorController.getRightX()) > 0.1;
                            return override ? operatorController.getLeftX() : driverController.getLeftX();
                        },
                        () -> {
                            var override =
                                    Math.abs(operatorController.getLeftX()) > 0.1 ||
                                            Math.abs(operatorController.getLeftY()) > 0.1 ||
                                            Math.abs(operatorController.getRightX()) > 0.1;
                            return -(override ? operatorController.getRightX() : driverController.getRightX());
                        }
                )
        );

        Intake.get().setDefaultCommand(Intake.get().pivotHover());
        Shooter.get().setDefaultCommand(Shooter.get().pivotHover());
    }

    private void configureButtonBindings() {
        operatorController.rightBumper().onTrue(Drive.get().resetRotationCommand());

        driverController.a().whileTrue(Intake.get().intake());

        driverController.b().toggleOnTrue(Commands.sequence(
                Shooter.get().pivotHover(),
                Intake.get().pivotHandoff(),
                Shooter.get().pivotHandoff(),
                Commands.race(
                        Intake.get().feedHandoff(),
                        Shooter.get().feedHandoff()
                ),
                Shooter.get().pivotShoot(),
                Intake.get().pivotHover(),
                Shooter.get().shootPercent(0.2, 0.75)
        ));

        operatorController.a().toggleOnTrue(Commands.parallel(
                Shooter.get().eject(),
                Intake.get().eject()
        ));

//        driverController.povUp().onTrue(Shooter.get().pivotHover());
//        driverController.povRight().onTrue(Shooter.get().pivotHandoff());
////        controller.povDown().onTrue(Shooter.get().pivotShoot());
//
//        driverController.a().onTrue(Intake.get().pivotHover());
//        driverController.b().onTrue(Intake.get().pivotHandoff());
////        controller.y().onTrue(Intake.get().pivotIntake());
//
//        driverController.y().toggleOnTrue(Intake.get().feed.setPercentCommand(0.3));
//        driverController.povLeft().toggleOnTrue(Intake.get().feed.setPercentCommand(-0.1));
//        driverController.povDown().toggleOnTrue(Shooter.get().feed.setPercentCommand(0.2));
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing.
        return autoChooser.get();
    }
}
