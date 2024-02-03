package frc.robot.subsystem.climber;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOValuesAutoLogged extends ClimberIO.ClimberIOValues implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ExtentionPositionLeft", extentionPositionLeft);
    table.put("ExtentionVelocityLeft", extentionVelocityLeft);
    table.put("ExtentionPositionRight", extentionPositionRight);
    table.put("ExtentionVelocityRight", extentionVelocityRight);
    table.put("AppliedVoltsLeft", appliedVoltsLeft);
    table.put("AppliedVoltsRight", appliedVoltsRight);
  }

  @Override
  public void fromLog(LogTable table) {
    extentionPositionLeft = table.get("ExtentionPositionLeft", extentionPositionLeft);
    extentionVelocityLeft = table.get("ExtentionVelocityLeft", extentionVelocityLeft);
    extentionPositionRight = table.get("ExtentionPositionRight", extentionPositionRight);
    extentionVelocityRight = table.get("ExtentionVelocityRight", extentionVelocityRight);
    appliedVoltsLeft = table.get("AppliedVoltsLeft", appliedVoltsLeft);
    appliedVoltsRight = table.get("AppliedVoltsRight", appliedVoltsRight);
  }

  public ClimberIOValuesAutoLogged clone() {
    ClimberIOValuesAutoLogged copy = new ClimberIOValuesAutoLogged();
    copy.extentionPositionLeft = this.extentionPositionLeft;
    copy.extentionVelocityLeft = this.extentionVelocityLeft;
    copy.extentionPositionRight = this.extentionPositionRight;
    copy.extentionVelocityRight = this.extentionVelocityRight;
    copy.appliedVoltsLeft = this.appliedVoltsLeft;
    copy.appliedVoltsRight = this.appliedVoltsRight;
    return copy;
  }
}
