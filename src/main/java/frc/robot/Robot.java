// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.command.defaults.Drive;
import frc.robot.sensor.pose.Odometry;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeIOSparkMax;
import frc.robot.subsystem.shooterV1.ShooterV1;
import frc.robot.subsystem.shooterV1.ShooterIOV1SparkMax;
import frc.robot.subsystem.swerve.Swerve;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  public static boolean hasNote;

  @Override
  public void robotInit() {

//    Pathfinding.setPathfinder(new LocalADStarAK());

    new Swerve();
//    Swerve.init();
    ShooterV1.init();
    Intake.init();
//    Climber.init();

    Swerve.instance.setDefaultCommand(new Drive());
    ShooterV1.instance.setDefaultCommand(Commands.idle(ShooterV1.instance));
    Intake.instance.setDefaultCommand(Commands.idle(Intake.instance));
//    Climber.instance.setDefaultCommand(Commands.idle(Climber.instance));

    ShooterIOV1SparkMax.paralyzedPivot = false;
    IntakeIOSparkMax.paralyzedDeploy = false;

    robotContainer = new RobotContainer();
//    Logger.addDataReceiver(new WPILOGWriter(Filesystem.getDeployDirectory().getPath() + "/logs"));
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    Swerve.instance.updateInputs();
    ShooterV1.instance.updateInputs();
    Intake.instance.updateInputs();
//    Climber.instance.updateInputs();

    // Periodic Actions
    CommandScheduler.getInstance().run();

    // LOG
    Odometry.log();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
