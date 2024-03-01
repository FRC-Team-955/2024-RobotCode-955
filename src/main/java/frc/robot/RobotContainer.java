// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command.*;
import frc.robot.command.handoff.IntakeHandoff;
import frc.robot.command.handoff.IntakeHandoffAuto;
import frc.robot.command.handoff.IntakeHandoffManual;
import frc.robot.command.handoff.IntakeHandoffTest;
import frc.robot.command.reset.AbortHandoff;
import frc.robot.command.reset.Reset;
import frc.robot.command.reset.SpitOutIntake;
import frc.robot.command.score.ScoreAmpManual;
import frc.robot.command.score.ScoreSubwooferManual;
import frc.robot.sensor.pose.Gyro;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.information.InputUtil;

import java.util.Optional;

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

//    registerAutoCommands();
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
    controller.rightBumper().onTrue(new ScoreAmpManual(controller.rightBumper()));
    controller.rightTrigger().onTrue(new ScoreSubwooferManual(controller.rightTrigger()));
//    controller.y().onTrue(new AbortHandoff());
//    controller.x().onTrue(new SpitOutIntake());
    controller.povUp().onTrue(Commands.runOnce(Gyro::resetGyro));
//
    controller2.rightTrigger().onTrue(new IntakeGroundManual(controller2.rightBumper()));
    controller2.leftTrigger().onTrue(new Handoff());
    controller2.rightTrigger().onTrue(new Spit());
    controller2.rightBumper().whileTrue(new FunctionalCommand(Intake::setIntakePercentIntake, ()->{},
            (b)->{Intake.setIntakePercent(0);},()->{return false;}, Intake.instance));
    Climber.instance.setDefaultCommand(new ClimbManual(() -> {
      return -InputUtil.deadzone(controller2.getLeftY(), 0.3);
    }));

    controller.povDown().onTrue(Commands.runOnce(CommandScheduler.getInstance()::cancelAll));
  }

  private void registerAutoCommands() {
    NamedCommands.registerCommand("intake", new IntakeHandoffAuto());
    NamedCommands.registerCommand("shoot", new ShootAuto());
    NamedCommands.registerCommand("prepShootSubwoofer", new PrepShootAuto(Constants.Shooter.Setpoints.subwoofer));
    NamedCommands.registerCommand("prepShootShort", new PrepShootAuto(Constants.Shooter.Setpoints.autoShootShort));
    NamedCommands.registerCommand("prepShootLong", new PrepShootAuto(Constants.Shooter.Setpoints.autoShootLong));
  }

  public Command getAutonomousCommand() {
    return Commands.idle();

//    return new SequentialCommandGroup(Commands.runOnce(Intake::movePositionHover, Intake.instance),
//            new WaitCommand(1),
//            Commands.runOnce(Shooter::setPivotPositionSubwoofer, Shooter.instance),
//            new WaitCommand(3),
//            Commands.runOnce(Shooter::shoot, Shooter.instance),
//            new WaitCommand(2),
//            Commands.runOnce(Shooter::setPivotPositionTuck, Shooter.instance),
//            new DriveCommand(0.5, 0, 0, 2));
  }

  public void rumbleControllers(boolean rumble) {
//    controllerRaw.setRumble(GenericHID.RumbleType.kBothRumble, 0.6);
  }
}
