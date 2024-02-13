package frc.robot.subsystem.intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Position", position);
    table.put("Velocity", velocity);
    table.put("NoteCaptured", noteCaptured);
    table.put("NoteSecured", noteSecured);
  }

  @Override
  public void fromLog(LogTable table) {
    position = table.get("Position", position);
    velocity = table.get("Velocity", velocity);
    noteCaptured = table.get("NoteCaptured", noteCaptured);
    noteSecured = table.get("NoteSecured", noteSecured);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.position = this.position;
    copy.velocity = this.velocity;
    copy.noteCaptured = this.noteCaptured;
    copy.noteSecured = this.noteSecured;
    return copy;
  }
}
