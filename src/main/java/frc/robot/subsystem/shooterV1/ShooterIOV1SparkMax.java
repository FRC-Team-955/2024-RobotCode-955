package frc.robot.subsystem.shooterV1;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.utility.object.MaxbotixUltrasonic;

public class ShooterIOV1SparkMax extends ShooterIOV1 {

    public static boolean paralyzedPivot = false;
    public static boolean paralyzedFeed = false;
    public static boolean paralyzedLeft = false;
    public static boolean paralyzedRight = false;

    private final CANSparkBase pivot;
    private final CANSparkBase feed;
    private final CANSparkBase flywheelLeft;
    private final CANSparkBase flywheelRight;
    private final RelativeEncoder pivotEncoder;
    private final RelativeEncoder feedEncoder;
    private final RelativeEncoder flywheelLeftEncoder;
    private final RelativeEncoder flywheelRightEncoder;
    private final MaxbotixUltrasonic ultrasonic;

    public ShooterIOV1SparkMax(ShooterIOInputsV1AutoLogged input) {
        inputs = input;
        pivot = new CANSparkMax(Constants.ShooterV1.pivotId, CANSparkLowLevel.MotorType.kBrushless);
        pivot.setIdleMode(CANSparkBase.IdleMode.kBrake);
        feed = new CANSparkMax(Constants.ShooterV1.feedId, CANSparkLowLevel.MotorType.kBrushless);
        flywheelLeft = new CANSparkMax(Constants.ShooterV1.flywheelLeftId, CANSparkLowLevel.MotorType.kBrushless);
        flywheelRight = new CANSparkMax(Constants.ShooterV1.flywheelRightId, CANSparkLowLevel.MotorType.kBrushless);
        pivotEncoder = pivot.getEncoder();
        feedEncoder = feed.getEncoder();
        flywheelLeftEncoder = flywheelLeft.getEncoder();
        flywheelRightEncoder = flywheelRight.getEncoder();
        pivotEncoder.setPositionConversionFactor(Constants.ShooterV1.gearRatioAngle * 360);
        ultrasonic = new MaxbotixUltrasonic(Constants.ShooterV1.ultrasonicId);
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
        inputs.ultrasonicRange = ultrasonic.getRangeInches();
    }

    @Override
    public void setPivotVolts(double volts) {
        inputs.voltsAppliedPivot = volts;
        pivot.setVoltage(paralyzedPivot ? 0 : MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setFeedVolts(double volts) {
        inputs.voltsAppliedFeed = volts;
        feed.setVoltage(paralyzedFeed ? 0 : MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setFlywheelLeftVolts(double volts) {
        inputs.voltsAppliedLeft = volts;
        flywheelLeft.setVoltage(paralyzedLeft ? 0 : MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setFlywheelRightVolts(double volts) {
        inputs.voltsAppliedRight = volts;
        flywheelRight.setVoltage(paralyzedRight ? 0 : -MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setFlywheelBrake(boolean brake) {
        inputs.brakeFlywheel = brake;
        flywheelLeft.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
        flywheelRight.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }
}
