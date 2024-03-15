// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.sensor.pose.Odometry;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeIOSparkMax;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.ShooterIOSparkMax;
import frc.robot.subsystem.swerve.Swerve;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  public static boolean hasNote;

  private void logConstantClass(Class<?> clazz, String parentName) {
    var parent = (parentName != null ? parentName + "." : "");
    for (var field : clazz.getFields()) {
      var key = parent + clazz.getSimpleName() + "." + field.getName();
      try {
        Logger.recordMetadata(key, field.get(null).toString());
      } catch (IllegalAccessException | IllegalArgumentException e) {
        Logger.recordMetadata(key, "Unknown");
      }
    }
    for (var subclass : clazz.getClasses()) {
      logConstantClass(subclass, parent + clazz.getSimpleName());
    }
  }

  @Override
  public void robotInit() {
    Logger.recordMetadata("* ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("* BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("* GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("* GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("* GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("* GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("* GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("* GitDirty", "Unknown");
        break;
    }
    logConstantClass(Constants.class, null);

    switch (Constants.mode) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Log to NetworkTables
        //SmartDashboard.putData("PowerDistribution", new PowerDistribution(Constants.pdhId, PowerDistribution.ModuleType.kRev)); // Enables power distribution logging
      }
      case SIM -> {
        Logger.addDataReceiver(new NT4Publisher());
      }
      case REPLAY -> {
        setUseTiming(!Constants.Simulation.replayRunAsFastAsPossible); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        if (!Constants.Simulation.replayRunAsFastAsPossible)
          Logger.addDataReceiver(new NT4Publisher()); // Log to NetworkTables if we are replaying in real time
      }
    }

    Logger.start();

    new Swerve();
//    Swerve.init();
    Shooter.init();
    Intake.init();
    Climber.init();

    Shooter.instance.setDefaultCommand(Commands.idle(Shooter.instance));
    Intake.instance.setDefaultCommand(Commands.run(Intake::movePositionHover, Intake.instance));
    Climber.instance.setDefaultCommand(Commands.idle(Climber.instance));

    Climber.setLeftBrake(true);
    Climber.setRightBrake(true);

    ShooterIOSparkMax.paralyzedPivot = false;
    ShooterIOSparkMax.paralyzedFeed = false;
    ShooterIOSparkMax.paralyzedFlywheel = false;
    IntakeIOSparkMax.paralyzedDeploy = false;
    IntakeIOSparkMax.paralyzedIntake = false;

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    Swerve.instance.updateInputs();
    Shooter.instance.updateInputs();
    Intake.instance.updateInputs();
    Climber.instance.updateInputs();

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

    Swerve.setBrakeMode(false);
    Intake.movePositionHover();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    Swerve.setBrakeMode(true);
  }

  @Override
  public void teleopInit() {
    Intake.movePositionHover();
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
