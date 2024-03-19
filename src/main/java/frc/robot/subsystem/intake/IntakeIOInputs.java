package frc.robot.subsystem.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputs implements LoggableInputs, Cloneable {

    public double deployPosition;
    public double deployVelocity;
    public double intakeVelocity;

    public boolean limitSwitch;

    public double deployPositionSetpoint;
    public double intakeVelocitySetpoint;

    public double voltsAppliedDeploy,   ampsAppliedDeploy;
    public double voltsAppliedIntake,   ampsAppliedIntake;

    @Override
    public void toLog(LogTable table) {
        table.put("DeployPosition",         deployPosition);
        table.put("DeployVelocity",         deployVelocity);
        table.put("IntakeVelocity",         intakeVelocity);

        table.put("LimitSwitch",            limitSwitch);

        table.put("DeployPositionSetpoint", deployPositionSetpoint);
        table.put("IntakeVelocitySetpoint", intakeVelocitySetpoint);

        table.put("VoltsAppliedDeploy",     voltsAppliedDeploy);
        table.put("AmpsAppliedDeploy",      ampsAppliedDeploy);
        table.put("VoltsAppliedIntake",     voltsAppliedIntake);
        table.put("AmpsAppliedIntake",      ampsAppliedIntake);
    }

    @Override
    public void fromLog(LogTable table) {
        deployPosition          = table.get("DeployPosition",           deployPosition);
        deployVelocity          = table.get("DeployVelocity",           deployVelocity);
        intakeVelocity          = table.get("IntakeVelocity",           intakeVelocity);

        limitSwitch             = table.get("LimitSwitch",              limitSwitch);

        deployPositionSetpoint  = table.get("DeployPositionSetpoint",   deployPositionSetpoint);
        intakeVelocitySetpoint  = table.get("IntakeVelocitySetpoint",   intakeVelocitySetpoint);

        voltsAppliedDeploy      = table.get("VoltsAppliedDeploy",       voltsAppliedDeploy);
        ampsAppliedDeploy       = table.get("AmpsAppliedDeploy",        ampsAppliedDeploy);
        voltsAppliedIntake      = table.get("VoltsAppliedIntake",       voltsAppliedIntake);
        ampsAppliedIntake       = table.get("AmpsAppliedIntake",        ampsAppliedIntake);
    }

    @Override
    public IntakeIOInputs clone() {
        IntakeIOInputs copy         = new IntakeIOInputs();

        copy.deployPosition         = this.deployPosition;
        copy.deployVelocity         = this.deployVelocity;
        copy.intakeVelocity         = this.intakeVelocity;

        copy.limitSwitch            = this.limitSwitch;

        copy.deployPositionSetpoint = this.deployPositionSetpoint;
        copy.intakeVelocitySetpoint = this.intakeVelocitySetpoint;

        copy.voltsAppliedDeploy     = this.voltsAppliedDeploy;
        copy.ampsAppliedDeploy      = this.ampsAppliedDeploy;
        copy.voltsAppliedIntake     = this.voltsAppliedIntake;
        copy.ampsAppliedIntake      = this.ampsAppliedIntake;

        return copy;
    }
}
