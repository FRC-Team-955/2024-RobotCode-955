// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command.*;
import frc.robot.command.factories.IntakeCommand;
import frc.robot.command.factories.ShooterCommand;
import frc.robot.command.handoff.HandoffFull;
import frc.robot.command.handoff.IntakeHandoffManual;
import frc.robot.sensor.pose.Gyro;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.information.InputUtil;

public class RobotContainer {

  CommandXboxController controller;
  CommandXboxController controller2;

  public RobotContainer() {
    controller = new CommandXboxController(0);
    controller2 = new CommandXboxController(1);
    configureBindings();
  }

  private void configureBindings() {
    Swerve.setPidHeadingControl(false);
    Swerve.instance.setDefaultCommand(Commands.run(() -> {
      Swerve.drivePercents(new Translation2d(-InputUtil.deadzone(controller.getLeftY(), 0.1),
              -InputUtil.deadzone(controller.getLeftX(), 0.1)),
              InputUtil.deadzone(controller.getRightX(), 0.1), true);
    }, Swerve.instance));
    controller.rightBumper().onTrue(new IntakeHandoffManual(controller.rightBumper()));
    controller.rightTrigger().onTrue(new ShootBasic(controller.rightTrigger()));
    controller.leftTrigger().onTrue(new AmpBasic(controller.leftTrigger()));
    controller.a().onTrue(Commands.runOnce(Shooter::setPivotPositionLoad));
    controller.b().onTrue(Commands.runOnce(Shooter::setPivotPositionAmp));
  }

  public void scuffed() {
//    Intake.setIntakePercent(InputUtil.deadzone(controller2.getRightY(), 0.1));
//    Climber.setVoltage(controller2.rightBumper().getAsBoolean() ? InputUtil.deadzone(controller2.getLeftY(), 0.1) * 12 : 0);
//    Shooter.setNotePosition(controller2.rightBumper().getAsBoolean() ? 0 : InputUtil.deadzone(controller2.getLeftY(), 0.1));
  }

  public Command getAutonomousCommand() {
    return Commands.idle();
//    return new SequentialCommandGroup(IntakeCommand.toHoverPosition(), new WaitCommand(1), new ShootBasic(), new DriveCommand(0.5, 0, 0, 2));
  }
}
