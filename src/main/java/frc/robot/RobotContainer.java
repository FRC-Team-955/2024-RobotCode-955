// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.sensor.pose.Odometry;
import frc.robot.subsystem.swerve.Swerve;

public class RobotContainer {

  CommandPS4Controller controller;

  public RobotContainer() {
    controller = new CommandPS4Controller(0);
    configureBindings();
  }

  private void configureBindings() {
    Swerve.instance.setDefaultCommand(Commands.run(() -> {
      Swerve.instance.drive(new Translation2d(controller.getLeftY() * 15, controller.getLeftX() * 15), controller.getRightX());
    }, Swerve.instance));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
