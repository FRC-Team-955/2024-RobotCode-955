package frc.robot.subsystem.shooter;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterIOInputsAutoLogged extends ShooterIO.ShooterIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PivotPosition", pivotPosition);
    table.put("PivotPositionSetpoint", pivotPositionSetpoint);
    table.put("PivotVelocity", pivotVelocity);
    table.put("FeedPosition", feedPosition);
    table.put("FeedVelocity", feedVelocity);
    table.put("FlywheelPositionTop", flywheelPositionTop);
    table.put("FlywheelVelocityTop", flywheelVelocityTop);
    table.put("FlywheelPositionBottom", flywheelPositionBottom);
    table.put("FlywheelVelocityBottom", flywheelVelocityBottom);
    table.put("BeamBreak", beamBreak);
    table.put("VoltsAppliedPivot", voltsAppliedPivot);
    table.put("VoltsAppliedFeed", voltsAppliedFeed);
    table.put("VoltsAppliedTop", voltsAppliedTop);
    table.put("VoltsAppliedBottom", voltsAppliedBottom);
    table.put("BrakeFlywheel", brakeFlywheel);
  }

  @Override
  public void fromLog(LogTable table) {
    pivotPosition = table.get("PivotPosition", pivotPosition);
    pivotPositionSetpoint = table.get("PivotPositionSetpoint", pivotPositionSetpoint);
    pivotVelocity = table.get("PivotVelocity", pivotVelocity);
    feedPosition = table.get("FeedPosition", feedPosition);
    feedVelocity = table.get("FeedVelocity", feedVelocity);
    flywheelPositionTop = table.get("FlywheelPositionTop", flywheelPositionTop);
    flywheelVelocityTop = table.get("FlywheelVelocityTop", flywheelVelocityTop);
    flywheelPositionBottom = table.get("FlywheelPositionBottom", flywheelPositionBottom);
    flywheelVelocityBottom = table.get("FlywheelVelocityBottom", flywheelVelocityBottom);
    beamBreak = table.get("BeamBreak", beamBreak);
    voltsAppliedPivot = table.get("VoltsAppliedPivot", voltsAppliedPivot);
    voltsAppliedFeed = table.get("VoltsAppliedFeed", voltsAppliedFeed);
    voltsAppliedTop = table.get("VoltsAppliedTop", voltsAppliedTop);
    voltsAppliedBottom = table.get("VoltsAppliedBottom", voltsAppliedBottom);
    brakeFlywheel = table.get("BrakeFlywheel", brakeFlywheel);
  }

  public ShooterIOInputsAutoLogged clone() {
    ShooterIOInputsAutoLogged copy = new ShooterIOInputsAutoLogged();
    copy.pivotPosition = this.pivotPosition;
    copy.pivotPositionSetpoint = this.pivotPositionSetpoint;
    copy.pivotVelocity = this.pivotVelocity;
    copy.feedPosition = this.feedPosition;
    copy.feedVelocity = this.feedVelocity;
    copy.flywheelPositionTop = this.flywheelPositionTop;
    copy.flywheelVelocityTop = this.flywheelVelocityTop;
    copy.flywheelPositionBottom = this.flywheelPositionBottom;
    copy.flywheelVelocityBottom = this.flywheelVelocityBottom;
    copy.beamBreak = this.beamBreak;
    copy.voltsAppliedPivot = this.voltsAppliedPivot;
    copy.voltsAppliedFeed = this.voltsAppliedFeed;
    copy.voltsAppliedTop = this.voltsAppliedTop;
    copy.voltsAppliedBottom = this.voltsAppliedBottom;
    copy.brakeFlywheel = this.brakeFlywheel;
    return copy;
  }
}
