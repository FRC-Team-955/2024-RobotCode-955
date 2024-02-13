package frc.robot.subsystem.shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Constants;

public class ShooterIOSparkMax extends ShooterIO {

    private final CANSparkBase pivot;
    private final CANSparkBase feed;
    private final CANSparkBase flywheelLeft;
    private final CANSparkBase flywheelRight;
    private final RelativeEncoder pivotEncoder;
    private final RelativeEncoder feedEncoder;
    private final RelativeEncoder flywheelLeftEncoder;
    private final RelativeEncoder flywheelRightEncoder;
//    private final Ultrasonic ultrasonic;

    public ShooterIOSparkMax(ShooterIOInputsAutoLogged input) {
        inputs = input;
        pivot = new CANSparkMax(Constants.Shooter.pivotId, CANSparkLowLevel.MotorType.kBrushless);
        feed = new CANSparkMax(Constants.Shooter.feedId, CANSparkLowLevel.MotorType.kBrushless);
        flywheelLeft = new CANSparkMax(Constants.Shooter.flywheelLeftId, CANSparkLowLevel.MotorType.kBrushless);
        flywheelRight = new CANSparkMax(Constants.Shooter.flywheelRightId, CANSparkLowLevel.MotorType.kBrushless);
        pivotEncoder = pivot.getEncoder();
        feedEncoder = feed.getEncoder();
        flywheelLeftEncoder = flywheelLeft.getEncoder();
        flywheelRightEncoder = flywheelRight.getEncoder();
        pivotEncoder.setPositionConversionFactor(Constants.Shooter.gearRatioAngle * 360);
//        ultrasonic = new Ultrasonic(Constants.Shooter.pingId, Constants.Shooter.echoId);
    }

    @Override
    public void updateInputs() {
        inputs.pivotPosition = pivotEncoder.getPosition();
        inputs.pivotVelocity = pivotEncoder.getVelocity();
        inputs.feedPosition = feedEncoder.getPosition();
        inputs.feedVelocity = feedEncoder.getVelocity();
        inputs.flywheelPositionLeft = flywheelLeftEncoder.getPosition();
        inputs.flywheelVelocityLeft = flywheelLeftEncoder.getVelocity();
        inputs.flywheelPositionRight = flywheelRightEncoder.getPosition();
        inputs.flywheelVelocityRight = flywheelRightEncoder.getVelocity();
        inputs.ultrasonicRange = 0;
//        inputs.ultrasonicRange = ultrasonic.getRangeInches();
    }

    @Override
    public void setPivotVolts(double volts) {
//        System.out.println(volts);
//        pivot.setVoltage(0);
        pivot.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setFeedVolts(double volts) {
        feed.setVoltage(0);
//        System.out.println(volts);
//        feed.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setFlywheelLeftVolts(double volts) {
//        flywheelLeft.setVoltage(0);
//        System.out.println(volts);
        flywheelLeft.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setFlywheelRightVolts(double volts) {
//        flywheelRight.setVoltage(0);
        flywheelRight.setVoltage(-MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setFlywheelBrake(boolean brake) {
        flywheelLeft.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
        flywheelRight.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }
}
