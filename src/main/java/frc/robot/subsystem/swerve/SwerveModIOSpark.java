package frc.robot.subsystem.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.utility.AngleUtil;

public class SwerveModIOSpark extends SwerveModIO {

    CANSparkBase drive;
    CANSparkBase angle;

    RelativeEncoder driveEncoder;
    RelativeEncoder angleEncoder;

    CANcoder absoluteEncoder;

    public SwerveModIOSpark(SwerveModIOInputsAutoLogged input, int id) {
        inputs = input;
        drive = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
        angle = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
        driveEncoder = drive.getEncoder();
        angleEncoder = angle.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveGearRatio * Constants.Swerve.relativeConversion);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveGearRatio * Constants.Swerve.relativeConversion);
        angleEncoder.setPositionConversionFactor(Constants.Swerve.angleGearRatio * Constants.Swerve.relativeConversion);
        angleEncoder.setVelocityConversionFactor(Constants.Swerve.angleGearRatio * Constants.Swerve.relativeConversion);
    }

    @Override
    public void updateInputs() {
        inputs.anglePositionAbsoluteDeg = absoluteEncoder.getPosition().getValue() * Constants.Swerve.absoluteConversion;
        inputs.anglePositionDeg = AngleUtil.unsignedRangeDegrees(angleEncoder.getPosition());
        inputs.angleVelocityDegSec = angleEncoder.getVelocity();
        inputs.drivePositionDeg = driveEncoder.getPosition();
        inputs.driveVelocityDegSec = driveEncoder.getVelocity();
    }

    @Override
    public void setDriveVolts(double volts) {
        drive.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setAngleVolts(double volts) {
        angle.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void syncEncoders() {
        angleEncoder.setPosition(inputs.anglePositionAbsoluteDeg);
    }

    @Override
    public void setIdleMode(SwerveMod.IdleMode mode) {
        CANSparkBase.IdleMode m = (mode == SwerveMod.IdleMode.Brake) ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast;
        drive.setIdleMode(m);
        angle.setIdleMode(m);
    }
}
