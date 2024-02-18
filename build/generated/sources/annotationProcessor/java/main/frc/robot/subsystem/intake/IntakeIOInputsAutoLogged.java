package frc.robot.subsystem.intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Position", position);
    table.put("PositionSetpoint", positionSetpoint);
    table.put("Velocity", velocity);
    table.put("IntakePercent", intakePercent);
    table.put("UltrasonicRange", ultrasonicRange);
    table.put("VoltsAppliedDeploy", voltsAppliedDeploy);
    table.put("VoltsAppliedIntake", voltsAppliedIntake);
    table.put("BrakeDeploy", brakeDeploy);
  }

  @Override
  public void fromLog(LogTable table) {
    position = table.get("Position", position);
    positionSetpoint = table.get("PositionSetpoint", positionSetpoint);
    velocity = table.get("Velocity", velocity);
    intakePercent = table.get("IntakePercent", intakePercent);
    ultrasonicRange = table.get("UltrasonicRange", ultrasonicRange);
    voltsAppliedDeploy = table.get("VoltsAppliedDeploy", voltsAppliedDeploy);
    voltsAppliedIntake = table.get("VoltsAppliedIntake", voltsAppliedIntake);
    brakeDeploy = table.get("BrakeDeploy", brakeDeploy);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.position = this.position;
    copy.positionSetpoint = this.positionSetpoint;
    copy.velocity = this.velocity;
    copy.intakePercent = this.intakePercent;
    copy.ultrasonicRange = this.ultrasonicRange;
    copy.voltsAppliedDeploy = this.voltsAppliedDeploy;
    copy.voltsAppliedIntake = this.voltsAppliedIntake;
    copy.brakeDeploy = this.brakeDeploy;
    return copy;
  }
}
