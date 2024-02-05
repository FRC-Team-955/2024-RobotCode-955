package frc.robot.subsystem.climber;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ExtensionPositionLeft", extensionPositionLeft);
    table.put("ExtensionVelocityLeft", extensionVelocityLeft);
    table.put("ExtensionPositionRight", extensionPositionRight);
    table.put("ExtensionVelocityRight", extensionVelocityRight);
  }

  @Override
  public void fromLog(LogTable table) {
    extensionPositionLeft = table.get("ExtensionPositionLeft", extensionPositionLeft);
    extensionVelocityLeft = table.get("ExtensionVelocityLeft", extensionVelocityLeft);
    extensionPositionRight = table.get("ExtensionPositionRight", extensionPositionRight);
    extensionVelocityRight = table.get("ExtensionVelocityRight", extensionVelocityRight);
  }

  public ClimberIOInputsAutoLogged clone() {
    ClimberIOInputsAutoLogged copy = new ClimberIOInputsAutoLogged();
    copy.extensionPositionLeft = this.extensionPositionLeft;
    copy.extensionVelocityLeft = this.extensionVelocityLeft;
    copy.extensionPositionRight = this.extensionPositionRight;
    copy.extensionVelocityRight = this.extensionVelocityRight;
    return copy;
  }
}
