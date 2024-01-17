package frc.robot.subsystem.swerve;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.utility.AngleUtil;

public class SwerveModIOSim extends SwerveModIO {

    private FlywheelSim drive = new FlywheelSim(DCMotor.getNEO(1), Constants.Swerve.driveGearRatio,Constants.Swerve.Simulation.driveMoi);
    private FlywheelSim angle = new FlywheelSim(DCMotor.getNEO(1), Constants.Swerve.angleGearRatio,Constants.Swerve.Simulation.angleMoi);

    private double anglePositionRel = 0;
    private double anglePositionAbs = 0;
    private double drivePosition = 0;

    private boolean brake;
    private double driveVolts;
    private double angleVolts;

    public SwerveModIOSim(SwerveModIOInputsAutoLogged inputs, int id) {

    }

    @Override
    public void updateInputs() {
        drive.update(0.02);
        angle.update(0.02);

        double driveVel = (brake && driveVolts == 0) ? 0 : AngleUtil.radToDeg(drive.getAngularVelocityRadPerSec());
        drivePosition += driveVel * 0.02;
        inputs.driveVelocityDegSec = driveVel;
        inputs.drivePositionDeg = drivePosition;

        double angleVel = (brake && angleVolts == 0) ? 0 : AngleUtil.radToDeg(angle.getAngularVelocityRadPerSec());
        anglePositionAbs += angleVel * 0.02;
        anglePositionRel += angleVel * 0.02;
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
