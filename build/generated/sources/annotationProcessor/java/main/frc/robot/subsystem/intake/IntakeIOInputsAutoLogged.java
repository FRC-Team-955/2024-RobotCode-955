package frc.robot.subsystem.intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Position", position);
    table.put("NoteDetected", noteDetected);
  }

  @Override
  public void fromLog(LogTable table) {
    position = table.get("Position", position);
    noteDetected = table.get("NoteDetected", noteDetected);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.position = this.position;
    copy.noteDetected = this.noteDetected;
    return copy;
  }
}
