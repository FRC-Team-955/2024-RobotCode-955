// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.swerve.Swerve;

public class RobotContainer {

  CommandXboxController controller;

  public RobotContainer() {
    controller = new CommandXboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    Swerve.setPidHeadingControl(false);
    Swerve.instance.setDefaultCommand(Commands.run(() -> {
      Swerve.drivePercents(new Translation2d(controller.getLeftY(), controller.getLeftX()), controller.getRightX(), false);
    }, Swerve.instance));
    controller.a().onTrue(Commands.runOnce(Swerve::syncEncoders));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
