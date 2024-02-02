package frc.robot.subsystem.swerve;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.utility.AngleUtil;

public class SwerveModIOSim extends SwerveModIO {

    private final FlywheelSim drive = new FlywheelSim(DCMotor.getNEO(1), Constants.Swerve.driveGearRatio,Constants.Swerve.Simulation.driveMoi);
    private final FlywheelSim angle = new FlywheelSim(DCMotor.getNEO(1), Constants.Swerve.angleGearRatio,Constants.Swerve.Simulation.angleMoi);

    private double anglePositionRel = 0;
    private double anglePositionAbs = 0;
    private double drivePosition = 0;

    private boolean brake;
    private double driveVolts;
    private double angleVolts;

    public SwerveModIOSim(SwerveModIOInputsAutoLogged input, int id) {
        inputs = input;
    }

    @Override
    public void updateInputs() {
        drive.update(0.02);
        angle.update(0.02);

        if ((brake && Math.abs(driveVolts) <= 0.25) || (DriverStation.isDisabled()))
            drive.setState(0);

        double driveVel = AngleUtil.radToDeg(drive.getAngularVelocityRadPerSec());
        drivePosition += driveVel * 0.02;
        inputs.driveVelocityDegSec = driveVel;
        inputs.drivePositionDeg = drivePosition;

        double angleVel = (brake && Math.abs(angleVolts) <= 0.25) || (DriverStation.isDisabled()) ? 0 : AngleUtil.radToDeg(angle.getAngularVelocityRadPerSec());
        anglePositionAbs = AngleUtil.unsignedRangeDegrees(anglePositionAbs + angleVel * 0.02);
        anglePositionRel = AngleUtil.unsignedRangeDegrees(anglePositionRel + angleVel * 0.02);
        inputs.anglePositionDeg = anglePositionRel;
        inputs.anglePositionAbsoluteDeg = anglePositionAbs;
        inputs.angleVelocityDegSec = angleVel;
    }

    @Override
    public void setDriveVolts(double volts) {
        driveVolts = MathUtil.clamp(volts, -12.0, 12.0);
        drive.setInputVoltage(driveVolts);
    }

    @Override
    public void setAngleVolts(double volts) {
        angleVolts = MathUtil.clamp(volts, -12.0, 12.0);
        angle.setInputVoltage(angleVolts);
    }

    @Override
    public void syncEncoders() {

    }

    @Override
    public void setIdleMode(SwerveMod.IdleMode mode) {
        brake = (mode == SwerveMod.IdleMode.Brake);
    }
}
