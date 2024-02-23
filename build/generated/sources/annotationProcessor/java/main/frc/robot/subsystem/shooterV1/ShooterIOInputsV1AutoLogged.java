package frc.robot.subsystem.shooterV1;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterIOInputsV1AutoLogged extends ShooterIOV1.ShooterIOInputsV1 implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PivotPosition", pivotPosition);
    table.put("PivotPositionSetpoint", pivotPositionSetpoint);
    table.put("PivotVelocity", pivotVelocity);
    table.put("FeedPosition", feedPosition);
    table.put("FeedVelocity", feedVelocity);
    table.put("FlywheelPositionLeft", flywheelPositionLeft);
    table.put("FlywheelVelocityLeft", flywheelVelocityLeft);
    table.put("FlywheelPositionRight", flywheelPositionRight);
    table.put("FlywheelVelocityRight", flywheelVelocityRight);
    table.put("UltrasonicRange", ultrasonicRange);
    table.put("VoltsAppliedPivot", voltsAppliedPivot);
    table.put("VoltsAppliedFeed", voltsAppliedFeed);
    table.put("VoltsAppliedLeft", voltsAppliedLeft);
    table.put("VoltsAppliedRight", voltsAppliedRight);
    table.put("BrakeFlywheel", brakeFlywheel);
  }

  @Override
  public void fromLog(LogTable table) {
    pivotPosition = table.get("PivotPosition", pivotPosition);
    pivotPositionSetpoint = table.get("PivotPositionSetpoint", pivotPositionSetpoint);
    pivotVelocity = table.get("PivotVelocity", pivotVelocity);
    feedPosition = table.get("FeedPosition", feedPosition);
    feedVelocity = table.get("FeedVelocity", feedVelocity);
    flywheelPositionLeft = table.get("FlywheelPositionLeft", flywheelPositionLeft);
    flywheelVelocityLeft = table.get("FlywheelVelocityLeft", flywheelVelocityLeft);
    flywheelPositionRight = table.get("FlywheelPositionRight", flywheelPositionRight);
    flywheelVelocityRight = table.get("FlywheelVelocityRight", flywheelVelocityRight);
    ultrasonicRange = table.get("UltrasonicRange", ultrasonicRange);
    voltsAppliedPivot = table.get("VoltsAppliedPivot", voltsAppliedPivot);
    voltsAppliedFeed = table.get("VoltsAppliedFeed", voltsAppliedFeed);
    voltsAppliedLeft = table.get("VoltsAppliedLeft", voltsAppliedLeft);
    voltsAppliedRight = table.get("VoltsAppliedRight", voltsAppliedRight);
    brakeFlywheel = table.get("BrakeFlywheel", brakeFlywheel);
  }

  public ShooterIOInputsV1AutoLogged clone() {
    ShooterIOInputsV1AutoLogged copy = new ShooterIOInputsV1AutoLogged();
    copy.pivotPosition = this.pivotPosition;
    copy.pivotPositionSetpoint = this.pivotPositionSetpoint;
    copy.pivotVelocity = this.pivotVelocity;
    copy.feedPosition = this.feedPosition;
    copy.feedVelocity = this.feedVelocity;
    copy.flywheelPositionLeft = this.flywheelPositionLeft;
    copy.flywheelVelocityLeft = this.flywheelVelocityLeft;
    copy.flywheelPositionRight = this.flywheelPositionRight;
    copy.flywheelVelocityRight = this.flywheelVelocityRight;
    copy.ultrasonicRange = this.ultrasonicRange;
    copy.voltsAppliedPivot = this.voltsAppliedPivot;
    copy.voltsAppliedFeed = this.voltsAppliedFeed;
    copy.voltsAppliedLeft = this.voltsAppliedLeft;
    copy.voltsAppliedRight = this.voltsAppliedRight;
    copy.brakeFlywheel = this.brakeFlywheel;
    return copy;
  }
}
