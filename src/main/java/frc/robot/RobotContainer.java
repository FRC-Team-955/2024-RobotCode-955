package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.lib.util.MotorFlags;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.EnumSet;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController driverController = Constants.Simulation.useNintendoSwitchProController ? new CommandNintendoSwitchProController(0) : new CommandXboxController(0);
    private final CommandXboxController operatorController = Constants.Simulation.useNintendoSwitchProController ? new CommandNintendoSwitchProController(1) : new CommandXboxController(1);

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices"); //AutoBuilder.buildAutoChooser());

    /* Subsystems */

    private Drive drive;
    private Intake intake;
    private Shooter shooter;

    public RobotContainer() {
        switch (Constants.mode) {
            case REAL -> {
                drive = new Drive(
                        // TODO: Figure out id
                        new VisionIOCamera("Front_Camera"),
                        new GyroIOPigeon2(10),
                        new ModuleIOSparkMax(0),
                        new ModuleIOSparkMax(1),
                        new ModuleIOSparkMax(2),
                        new ModuleIOSparkMax(3)
                );
                intake = new Intake(
                        new IntakeIO(),
                        new ArmIOSparkMax(3),
                        new AbsoluteEncoderIO(),
                        new WheelIOSparkMax(16, EnumSet.of(MotorFlags.Inverted))
                );
                shooter = new Shooter(
                        new ShooterIOReal(6),
                        new ArmIOSparkMax(7),
                        new AbsoluteEncoderIOREVThroughBore(0),
                        new WheelIOSparkMax(9, EnumSet.noneOf(MotorFlags.class)),
                        new WheelIOSparkMax(10, EnumSet.of(MotorFlags.Inverted)),
                        new WheelIOSparkMax(8, EnumSet.of(MotorFlags.Inverted))
                );
            }

            case SIM -> {
                drive = new Drive(
                        new VisionIO(),
                        new GyroIO(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim()
                );
                intake = new Intake(
                        new IntakeIO(),
                        new ArmIOSim(DCMotor.getNEO(1), 0.3, 0.045),
                        new AbsoluteEncoderIOSim(),
                        new WheelIOSim(DCMotor.getNEO(1))
                );
                shooter = new Shooter(
                        new ShooterIOSim(),
                        new ArmIOSim(DCMotor.getNEO(1), 0.4, 0.083),
                        new AbsoluteEncoderIOSim(),
                        new WheelIOSim(DCMotor.getNEO(1)),
                        new WheelIOSim(DCMotor.getNEO(1)),
                        new WheelIOSim(DCMotor.getNEO(1))
                );
            }

            case REPLAY -> {
                drive = new Drive(
                        new VisionIO(),
                        new GyroIO(),
                        new ModuleIO(),
                        new ModuleIO(),
                        new ModuleIO(),
                        new ModuleIO()
                );
                intake = new Intake(new IntakeIO(), new ArmIO(), new AbsoluteEncoderIO(), new WheelIO());
                shooter = new Shooter(new ShooterIO(), new ArmIO(), new AbsoluteEncoderIO(), new WheelIO(), new WheelIO(), new WheelIO());
            }
        }

        // Check fields and warn if subsystems are uninitialized
        for (var field : getClass().getDeclaredFields()) {
            try {
                if (field.get(this) == null) {
                    var msg = "RobotContainer." + field.getName() + " is null. If the field is a subsystem please fix it";
                    if (Constants.mode.isSim()) {
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
//        NamedCommands.registerCommand("Run Flywheel", Flywheel.get().run());

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
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
                drive.joystickDrive(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
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

    private final LoggedDashboardNumber shootPercent = new LoggedDashboardNumber("Shoot Percent", 0.3);
    private final LoggedDashboardNumber shootSpinupTime = new LoggedDashboardNumber("Shoot Spinup Time", 0.5);

    private void configureButtonBindings() {
        driverController.a().onTrue(drive.resetRotationCommand());

        driverController.leftTrigger(0.25).whileTrue(
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

        // b for handoff
        driverController.b().toggleOnTrue(Commands.sequence(
                // handoff if shooter doesn't have a note
                Commands.either(
                        Commands.none(),
                        Commands.sequence(
                                shooter.pivotWaitForIntake(),
                                intake.pivotHandoff(),
                                shooter.pivotHandoff(),
                                Commands.race(
                                        intake.feedHandoff(),
                                        shooter.feedHandoff()
                                )
                        ),
                        shooter::hasNoteDebounced
                ),
                shooter.pivotWaitForIntake(),
                intake.pivotHover()
        ));
        
        driverController.rightBumper().toggleOnTrue(Commands.sequence(
                shooter.pivotShoot(),
                shooter.shootPercent(0.5, 0.6)
        ));
        
        driverController.rightTrigger(0.25).whileTrue(Commands.sequence(
                shooter.pivotShoot(),
                shooter.shootPercentUntimed(0.5)
        ).finallyDo(() -> shooter.shootPercent(0.5, 0).schedule())
        );

        driverController.x().toggleOnTrue(Commands.parallel(
                shooter.eject(),
                intake.eject()
        ));
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing.
        return autoChooser.get();
    }
}
