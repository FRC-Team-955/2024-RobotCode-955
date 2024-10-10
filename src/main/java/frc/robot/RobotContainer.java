package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.factories.FourPieceWingAutoFactory;
import frc.robot.factories.HandoffFactory;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMaxBeamBreak;
import frc.robot.util.CommandNintendoSwitchProController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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
    private Drive drive;
    private Intake intake;
    private Shooter shooter;

    public RobotContainer() {
        switch (Constants.mode) {
            case REAL -> {
                drive = new Drive(
                        new GyroIOPigeon2(10),
                        new ModuleIOSparkMaxCANcoder(0),
                        new ModuleIOSparkMaxCANcoder(1),
                        new ModuleIOSparkMaxCANcoder(2),
                        new ModuleIOSparkMaxCANcoder(3),
                        new VisionIOCamera("Shooter_Cam")
                );
                intake = new Intake(new IntakeIOSparkMax(3, 16));
                shooter = new Shooter(new ShooterIOSparkMaxBeamBreak(6, 7, 9, 10, 8));
            }

            case SIM -> {
                drive = new Drive(
                        new GyroIO(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new VisionIO()
                );
                intake = new Intake(new IntakeIOSim(DCMotor.getNEO(1), 0.3, 0.045, DCMotor.getNEO(1)));
                shooter = new Shooter(new ShooterIOSim(/*DCMotor.getNEO(1), 0.4, 0.083, DCMotor.getNEO(1), DCMotor.getNEO(1), DCMotor.getNEO(1)*/));
            }

            case REPLAY -> {
                drive = new Drive(
                        new GyroIO(),
                        new ModuleIO(),
                        new ModuleIO(),
                        new ModuleIO(),
                        new ModuleIO(),
                        new VisionIO()
                );
                intake = new Intake(new IntakeIO());
                shooter = new Shooter(new ShooterIO());
            }
        }

        // Check fields and warn if subsystems are uninitialized
        for (var field : getClass().getDeclaredFields()) {
            try {
                if (field.get(this) == null) {
                    var msg = "RobotContainer." + field.getName() + " is null. If the field is a subsystem please fix it";
                    if (Constants.isSim) {
                        throw new RuntimeException(msg);
                    } else {
                        for (int i = 0; i < 3; i++) {
                            // Without the i the driver station will recognize duplicate messages
                            DriverStation.reportError("(" + (i + 1) + ") " + msg, false);
                        }
                    }
                }
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }

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

        autoChooser.addOption("Shoot Configurable", shooter.shootConfigurable());

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
                drive.joystickDrive(
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

        driverController.leftTrigger(0.25).toggleOnTrue(
                //shooter.shootConfigurable()
                Commands.deferredProxy(
                        () -> drive.disableVision.get() ?
                        shooter.shoot() :
                        shooter.shootDistance(drive.distanceToSpeaker())
                )
        );

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

        driverController.b().toggleOnTrue(HandoffFactory.get());
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing.
        return autoChooser.get();
    }
}
