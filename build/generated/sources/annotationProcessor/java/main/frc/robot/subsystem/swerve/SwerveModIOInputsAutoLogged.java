package frc.robot.subsystem.swerve;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModIOInputsAutoLogged extends SwerveModIO.SwerveModIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DrivePositionDeg", drivePositionDeg);
    table.put("DriveVelocityDegSec", driveVelocityDegSec);
    table.put("AnglePositionDeg", anglePositionDeg);
    table.put("AnglePositionAbsoluteDeg", anglePositionAbsoluteDeg);
    table.put("AngleVelocityDegSec", angleVelocityDegSec);
  }

  @Override
  public void fromLog(LogTable table) {
    drivePositionDeg = table.get("DrivePositionDeg", drivePositionDeg);
    driveVelocityDegSec = table.get("DriveVelocityDegSec", driveVelocityDegSec);
    anglePositionDeg = table.get("AnglePositionDeg", anglePositionDeg);
    anglePositionAbsoluteDeg = table.get("AnglePositionAbsoluteDeg", anglePositionAbsoluteDeg);
    angleVelocityDegSec = table.get("AngleVelocityDegSec", angleVelocityDegSec);
  }

  public SwerveModIOInputsAutoLogged clone() {
    SwerveModIOInputsAutoLogged copy = new SwerveModIOInputsAutoLogged();
    copy.drivePositionDeg = this.drivePositionDeg;
    copy.driveVelocityDegSec = this.driveVelocityDegSec;
    copy.anglePositionDeg = this.anglePositionDeg;
    copy.anglePositionAbsoluteDeg = this.anglePositionAbsoluteDeg;
    copy.angleVelocityDegSec = this.angleVelocityDegSec;
    return copy;
  }
}
