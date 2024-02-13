// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command.factories.ShooterCommand;
import frc.robot.sensor.pose.Gyro;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.information.InputUtil;

public class RobotContainer {

  CommandXboxController controller;

  public RobotContainer() {
    controller = new CommandXboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    Swerve.setPidHeadingControl(false);
    Swerve.instance.setDefaultCommand(Commands.run(() -> {
      Swerve.drivePercents(new Translation2d(-InputUtil.deadzone(controller.getLeftY(), 0.1),
              -InputUtil.deadzone(controller.getLeftX(), 0.1)),
              InputUtil.deadzone(controller.getRightX(), 0.1), true);
    }, Swerve.instance));
    controller.a().onTrue(Commands.runOnce(Shooter::setPivotPositionLoad));
    controller.b().onTrue(Commands.runOnce(() -> {
      Shooter.setPivotPosition(50);
    }));
    controller.x().onTrue(Commands.runOnce(Shooter::setFlywheelVelocityMax));
    controller.y().onTrue(Commands.runOnce(Shooter::setFlywheelVelocityZero));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
