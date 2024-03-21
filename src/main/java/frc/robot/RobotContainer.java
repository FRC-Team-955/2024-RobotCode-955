// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command.*;
import frc.robot.sensor.pose.Gyro;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.information.InputUtil;

public class RobotContainer {

  public static RobotContainer instance;

  CommandXboxController controller;
  XboxController controllerRaw;
  CommandXboxController controller2;
  XboxController controller2Raw;

  public RobotContainer() {
    instance = this;

    controller = new CommandXboxController(0);
    controller2 = new CommandXboxController(1);
    controllerRaw = new XboxController(0);
    controller2Raw = new XboxController(1);

    controllerRaw.setRumble(GenericHID.RumbleType.kBothRumble, 0);

    registerAutoCommands();
    configureBindings();

  }

  private void configureBindings() {
//    Swerve.setPidHeadingControl(true);
    Swerve.setPidHeadingControl(false);
    Swerve.instance.setDefaultCommand(Commands.run(() -> {
      Swerve.drivePercents(new Translation2d(-InputUtil.deadzone(controller.getLeftY(), 0.1),
              -InputUtil.deadzone(controller.getLeftX(), 0.1)),
              -InputUtil.deadzone(controller.getRightX(), 0.1), true);
    }, Swerve.instance));

//    controller.rightBumper().onTrue(AutoAlign.amp());
//    controller.rightTrigger().onTrue(AutoAlign.subwoofer());
//    controller.rightBumper().onTrue(new ScoreAmpManual(controller.rightBumper()));
//    controller.rightTrigger().onTrue(new ScoreSpeakerManual(controller.rightTrigger()));
//    controller.y().onTrue(new AbortHandoff());
//    controller.x().onTrue(new SpitOutIntake());
    controller.povUp().onTrue(Commands.runOnce(Gyro::resetGyro));
//    controller.leftTrigger().onTrue(new IntakeSource(controller.leftTrigger()));
////    controller.leftTrigger().onTrue(AutoAlign.align(new Pose2d(1.3321382999420166, 5.586578369140625,
////            Rotation2d.fromRadians(0.0))));
//
//    controller2.rightTrigger().onTrue(new IntakeGroundManual(controller2.rightTrigger()));
//    controller2.leftTrigger().onTrue(new Handoff());
//    controller2.rightBumper().onTrue(new Spit());
//    controller2.leftBumper().onTrue(new SequentialCommandGroup(Commands.runOnce(() -> { Shooter.setSpinup(true); }),
//            new WaitCommand(1), Commands.runOnce(() -> { Shooter.setSpinup(false); })));
    Climber.instance.setDefaultCommand(new ClimbManual(() -> {
      return -InputUtil.deadzone(controller2.getLeftY(), 0.3);
    }));



    controller.povDown().onTrue(Commands.runOnce(CommandScheduler.getInstance()::cancelAll));
    controller.povDown().onTrue(Commands.runOnce(() -> {
//      Shooter.setSpinup(false);
//      Shooter.setAmpSpinup(false);
//      Intake.setIntakePercent(0);
//      Shooter.setIntaking(false);
//      Shooter.setIntakingSource(false);
    }));
//    controller.povDown().onTrue(new Reset());
  }

  private void registerAutoCommands() {
//    NamedCommands.registerCommand("intake", new IntakeHandoff());
//    NamedCommands.registerCommand("shoot", new ShootAuto());
//    NamedCommands.registerCommand("prepShootSubwoofer", new PrepShootAuto(Constants.Shooter.Setpoints.subwoofer));
//    NamedCommands.registerCommand("prepShootShort", new PrepShootAuto(Constants.Shooter.Setpoints.autoShootShort));
//    NamedCommands.registerCommand("prepShootLong", new PrepShootAuto(Constants.Shooter.Setpoints.autoShootLong));
    NamedCommands.registerCommand("checkAlign", AutoAlign.alignCorrect(new Pose2d(1.3321382999420166, 5.586578369140625,
            Rotation2d.fromRadians(0.0))));
  }

  public Command getAutonomousCommand() {
    return Commands.idle();
  }

  public void rumbleControllers(boolean rumble) {
    //controllerRaw.setRumble(GenericHID.RumbleType.kBothRumble, 0.6);
  }
}
