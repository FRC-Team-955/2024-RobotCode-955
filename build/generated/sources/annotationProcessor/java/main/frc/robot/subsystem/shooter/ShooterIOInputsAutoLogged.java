package frc.robot.subsystem.shooter;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterIOInputsAutoLogged extends ShooterIO.ShooterIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PivotPosition", pivotPosition);
    table.put("PivotVelocity", pivotVelocity);
    table.put("FeedPosition", feedPosition);
    table.put("FeedVelocity", feedVelocity);
    table.put("FlywheelPositionLeft", flywheelPositionLeft);
    table.put("FlywheelVelocityLeft", flywheelVelocityLeft);
    table.put("FlywheelPositionRight", flywheelPositionRight);
    table.put("FlywheelVelocityRight", flywheelVelocityRight);
    table.put("UltrasonicRange", ultrasonicRange);
  }

  @Override
  public void fromLog(LogTable table) {
    pivotPosition = table.get("PivotPosition", pivotPosition);
    pivotVelocity = table.get("PivotVelocity", pivotVelocity);
    feedPosition = table.get("FeedPosition", feedPosition);
    feedVelocity = table.get("FeedVelocity", feedVelocity);
    flywheelPositionLeft = table.get("FlywheelPositionLeft", flywheelPositionLeft);
    flywheelVelocityLeft = table.get("FlywheelVelocityLeft", flywheelVelocityLeft);
    flywheelPositionRight = table.get("FlywheelPositionRight", flywheelPositionRight);
    flywheelVelocityRight = table.get("FlywheelVelocityRight", flywheelVelocityRight);
    ultrasonicRange = table.get("UltrasonicRange", ultrasonicRange);
  }

  public ShooterIOInputsAutoLogged clone() {
    ShooterIOInputsAutoLogged copy = new ShooterIOInputsAutoLogged();
    copy.pivotPosition = this.pivotPosition;
    copy.pivotVelocity = this.pivotVelocity;
    copy.feedPosition = this.feedPosition;
    copy.feedVelocity = this.feedVelocity;
    copy.flywheelPositionLeft = this.flywheelPositionLeft;
    copy.flywheelVelocityLeft = this.flywheelVelocityLeft;
    copy.flywheelPositionRight = this.flywheelPositionRight;
    copy.flywheelVelocityRight = this.flywheelVelocityRight;
    copy.ultrasonicRange = this.ultrasonicRange;
    return copy;
  }
}
