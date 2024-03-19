package frc.robot.subsystem.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterIOInputs implements LoggableInputs, Cloneable {

    public double   pivotPosition;
    public double   pivotVelocity;
    public double   feedVelocity;
    public double   flywheelVelocityTop;
    public double   flywheelVelocityBottom;

    public boolean  beamBreak;

    public double   pivotPositionSetpoint;
    public double   feedVelocitySetpoint;
    public double   flywheelVelocitySetpoint;

    public double   voltsAppliedPivot,            ampsAppliedPivot;
    public double   voltsAppliedFeed,             ampsAppliedFeed;
    public double   voltsAppliedFlywheelTop,      ampsAppliedFlywheelTop;
    public double   voltsAppliedFlywheelBottom,   ampsAppliedFlywheelBottom;

    @Override
    public void toLog(LogTable table) {
        table.put("PivotPosition",              pivotPosition);
        table.put("PivotVelocity",              pivotVelocity);
        table.put("FeedVelocity",               feedVelocity);
        table.put("FlywheelVelocityTop",        flywheelVelocityTop);
        table.put("FlywheelVelocityBottom",     flywheelVelocityBottom);

        table.put("BeamBreak",                  beamBreak);

        table.put("PivotPositionSetpoint",      pivotPositionSetpoint);
        table.put("FeedVelocitySetpoint",       feedVelocitySetpoint);
        table.put("FlywheelVelocitySetpoint",   flywheelVelocitySetpoint);

        table.put("VoltsAppliedPivot",          voltsAppliedPivot);
        table.put("AmpsAppliedPivot",           ampsAppliedPivot);
        table.put("VoltsAppliedFeed",           voltsAppliedFeed);
        table.put("AmpsAppliedFeed",            ampsAppliedFeed);
        table.put("VoltsAppliedFlywheelTop",    voltsAppliedFlywheelTop);
        table.put("AmpsAppliedFlywheelTop",     ampsAppliedFlywheelTop);
        table.put("VoltsAppliedFlywheelBottom", voltsAppliedFlywheelBottom);
        table.put("AmpsAppliedFlywheelBottom",  ampsAppliedFeed);
    }

    @Override
    public void fromLog(LogTable table) {
        pivotPosition               = table.get("PivotPosition",                pivotPosition);
        pivotVelocity               = table.get("PivotVelocity",                pivotVelocity);
        feedVelocity                = table.get("FeedVelocity",                 feedVelocity);
        flywheelVelocityTop         = table.get("FlywheelVelocityTop",          flywheelVelocityTop);
        flywheelVelocityBottom      = table.get("FlywheelVelocityBottom",       flywheelVelocityBottom);

        beamBreak                   = table.get("BeamBreak",                    beamBreak);

        pivotPositionSetpoint       = table.get("PivotPositionSetpoint",        pivotPositionSetpoint);
        feedVelocitySetpoint        = table.get("FeedVelocitySetpoint",         feedVelocitySetpoint);
        flywheelVelocitySetpoint    = table.get("FlywheelVelocitySetpoint",     flywheelVelocitySetpoint);

        voltsAppliedPivot           = table.get("VoltsAppliedPivot",            voltsAppliedPivot);
        ampsAppliedPivot            = table.get("AmpsAppliedPivot",             ampsAppliedPivot);
        voltsAppliedFeed            = table.get("VoltsAppliedFeed",             voltsAppliedFeed);
        ampsAppliedFeed             = table.get("AmpsAppliedFeed",              ampsAppliedFeed);
        voltsAppliedFlywheelTop     = table.get("VoltsAppliedFlywheelTop",      voltsAppliedFlywheelTop);
        ampsAppliedFlywheelTop      = table.get("AmpsAppliedFlywheelTop",       ampsAppliedFlywheelTop);
        voltsAppliedFlywheelBottom  = table.get("VoltsAppliedFlywheelBottom",   voltsAppliedFlywheelBottom);
        ampsAppliedFlywheelBottom   = table.get("AmpsAppliedFlywheelBottom",    ampsAppliedFlywheelBottom);
    }

    @Override
    public ShooterIOInputs clone() {
        ShooterIOInputs copy            = new ShooterIOInputs();

        copy.pivotPosition              = this.pivotPosition;
        copy.pivotVelocity              = this.pivotVelocity;
        copy.feedVelocity               = this.feedVelocity;
        copy.flywheelVelocityTop        = this.flywheelVelocityTop;
        copy.flywheelVelocityBottom     = this.flywheelVelocityBottom;

        copy.beamBreak                  = this.beamBreak;

        copy.pivotPositionSetpoint      = this.pivotPositionSetpoint;
        copy.feedVelocitySetpoint       = this.feedVelocitySetpoint;
        copy.flywheelVelocitySetpoint   = this.flywheelVelocitySetpoint;

        copy.voltsAppliedPivot          = this.voltsAppliedPivot;
        copy.ampsAppliedPivot           = this.ampsAppliedPivot;
        copy.voltsAppliedFeed           = this.voltsAppliedFeed;
        copy.ampsAppliedFeed            = this.ampsAppliedFeed;
        copy.voltsAppliedFlywheelTop    = this.voltsAppliedFlywheelTop;
        copy.ampsAppliedFlywheelTop     = this.ampsAppliedFlywheelTop;
        copy.voltsAppliedFlywheelBottom = this.voltsAppliedFlywheelBottom;
        copy.ampsAppliedFlywheelBottom  = this.ampsAppliedFlywheelBottom;

        return copy;
    }
}
