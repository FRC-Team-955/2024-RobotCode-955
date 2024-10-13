package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.factories.CalculatedShootFactory;
import frc.robot.factories.FourPieceWingAutoFactory;
import frc.robot.factories.HandoffFactory;
import frc.robot.factories.ThreePieceMidlineAutoFactory;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.CommandNintendoSwitchProController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController driverController = Constants.Simulation.useNintendoSwitchProController ? new CommandNintendoSwitchProController(0) : new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices"); //AutoBuilder.buildAutoChooser());

    /* Subsystems */
    private final Drive drive = Drive.get();
    private final Intake intake = Intake.get();
    private final Shooter shooter = Shooter.get();

    public RobotContainer() {
        addAutos();
        setDefaultCommands();
        configureButtonBindings();

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    }

    private void addAutos() {
        autoChooser.addOption(
                "Mobility (robot relative forward)",
                drive.driveVelocity(new ChassisSpeeds(2, 0, 0), 3)
        );

        autoChooser.addOption("none", null);

        autoChooser.addOption("shoot & move",
                Commands.sequence(
                        intake.pivotHover(),
                        shooter.pivotShoot(),
//                        shooter.shootPercent(0.5, 0.6),
                        drive.driveVelocity(new ChassisSpeeds(2, 0, 0), 3)
                )
        );

        NamedCommands.registerCommand("shoot", Commands.print("test").andThen(Commands.waitSeconds(2)));

        var factory = drive.createAutoFactory();
        autoChooser.addDefaultOption("4 Piece Wing", FourPieceWingAutoFactory.get(factory));
        autoChooser.addDefaultOption("3 Piece Midline", ThreePieceMidlineAutoFactory.get(factory));

        // Set up SysId routines
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );

        autoChooser.addOption(
                "Shooter Pivot SysId (Quasistatic Forward)",
                shooter.pivotSysId.quasistatic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Shooter Pivot SysId (Quasistatic Reverse)",
                shooter.pivotSysId.quasistatic(SysIdRoutine.Direction.kReverse)
        );
        autoChooser.addOption(
                "Shooter Pivot SysId (Dynamic Forward)",
                shooter.pivotSysId.dynamic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Shooter Pivot SysId (Dynamic Reverse)",
                shooter.pivotSysId.dynamic(SysIdRoutine.Direction.kReverse)
        );

        autoChooser.addOption(
                "Shooter Feed SysId (Quasistatic Forward)",
                shooter.feedSysId.quasistatic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Shooter Feed SysId (Quasistatic Reverse)",
                shooter.feedSysId.quasistatic(SysIdRoutine.Direction.kReverse)
        );
        autoChooser.addOption(
                "Shooter Feed SysId (Dynamic Forward)",
                shooter.feedSysId.dynamic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Shooter Feed SysId (Dynamic Reverse)",
                shooter.feedSysId.dynamic(SysIdRoutine.Direction.kReverse)
        );

        autoChooser.addOption(
                "Shooter Flywheels SysId (Quasistatic Forward)",
                shooter.flywheelsSysId.quasistatic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Shooter Flywheels SysId (Quasistatic Reverse)",
                shooter.flywheelsSysId.quasistatic(SysIdRoutine.Direction.kReverse)
        );
        autoChooser.addOption(
                "Shooter Flywheels SysId (Dynamic Forward)",
                shooter.flywheelsSysId.dynamic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
                "Shooter Flywheels SysId (Dynamic Reverse)",
                shooter.flywheelsSysId.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }

    private void setDefaultCommands() {
        //noinspection SuspiciousNameCombination
        drive.setDefaultCommand(
                drive.driveJoystick(
                        driverController::getLeftY,
                        driverController::getLeftX,
                        () -> -driverController.getRightX()
                )
        );

        intake.setDefaultCommand(intake.pivotHover());
        shooter.setDefaultCommand(Commands.either(
                shooter.pivotHover(),
                shooter.pivotWaitForIntake(),
                () -> intake.isClearOfShooter() || shooter.alreadyAtHover()
        ));
    }

    private void configureButtonBindings() {
        driverController.y().onTrue(drive.resetRotationCommand());

        driverController.rightTrigger(0.25).whileTrue(
                intake.intake()
                // auto handoff after intake
                //        .finallyDo(() -> Commands.sequence(
                //                        shooter.pivotWaitForIntake(),
                //                        intake.pivotHandoff(),
                //                        shooter.pivotHandoff(),
                //                        Commands.race(
                //                                intake.feedHandoff(),
                //                                shooter.feedHandoff()
                //                        ),
                //                        shooter.pivotWaitForIntake(),
                //                        intake.pivotHover(),
                //                        shooter.pivotHover()
                //                )
                //                .withTimeout(3)
                //                .schedule())
        );
        /*
        driverController.leftTrigger(0.25).toggleOnTrue(shooter.shoot());
        */

        driverController.leftTrigger(0.25).toggleOnTrue(CalculatedShootFactory.get(driverController::getLeftY, driverController::getLeftX));

        driverController.leftBumper().toggleOnTrue(shooter.amp());

//        driverController.leftTrigger(0.25).whileTrue(Commands.sequence(
//                        shooter.pivotShoot(),
//                        shooter.shootPercentUntimed(0.5)
//                ).finallyDo(() -> shooter.shootPercent(0.5, 0).schedule())
//        );

        driverController.x().toggleOnTrue(Commands.parallel(
                shooter.eject(),
                intake.eject()
        ).withTimeout(1));

        driverController.rightBumper().toggleOnTrue(HandoffFactory.get());
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing.
        return autoChooser.get();
    }
}
