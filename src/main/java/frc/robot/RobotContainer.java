// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command.*;
import frc.robot.command.handoff.IntakeHandoff;
import frc.robot.command.handoff.IntakeHandoffAuto;
import frc.robot.command.handoff.IntakeHandoffManual;
import frc.robot.subsystem.shooterV1.ShooterV1;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.information.InputUtil;

public class RobotContainer {

  CommandXboxController controller;
  CommandXboxController controller2;

  public RobotContainer() {
    controller = new CommandXboxController(0);
    controller2 = new CommandXboxController(1);

    registerAutoCommands();
    configureBindings();

  }

  private void configureBindings() {
    Swerve.setPidHeadingControl(false);
    Swerve.instance.setDefaultCommand(Commands.run(() -> {
      Swerve.drivePercents(new Translation2d(-InputUtil.deadzone(controller.getLeftY(), 0.1),
              -InputUtil.deadzone(controller.getLeftX(), 0.1)),
              -InputUtil.deadzone(controller.getRightX(), 0.1), true);
    }, Swerve.instance));
    controller.rightBumper().onTrue(new IntakeHandoffManual(controller.rightBumper()));
    controller.rightTrigger().onTrue(new ShootBasic(controller.rightTrigger()));
    controller.leftTrigger().onTrue(new AmpBasic(controller.leftTrigger()));
    controller.leftBumper().onTrue(new TrapBasic(controller.leftBumper()));
    controller.a().onTrue(Commands.runOnce(ShooterV1::setPivotPositionLoad));
    controller.b().onTrue(Commands.runOnce(ShooterV1::setPivotPositionAmp));
  }

  private void registerAutoCommands() {
    NamedCommands.registerCommand("intake", new IntakeHandoffAuto());
    NamedCommands.registerCommand("shoot", new ShootAuto());
    NamedCommands.registerCommand("prepShootSubwoofer", new PrepShootSubwooferAuto());
    NamedCommands.registerCommand("prepShootShort", new PrepShootShortAuto());
    NamedCommands.registerCommand("prepShootLong", new PrepShootLongAuto());
  }

  public Command getAutonomousCommand() {
    return Commands.idle();
//    return new SequentialCommandGroup(IntakeCommand.toHoverPosition(), new WaitCommand(1), new ShootBasic(), new DriveCommand(0.5, 0, 0, 2));
  }
}
