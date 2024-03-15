package frc.robot.subsystem.shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ShooterIOSparkMax extends ShooterIO {

    public static boolean paralyzedPivot = false;
    public static boolean paralyzedFeed = false;
    public static boolean paralyzedFlywheel = false;

    private final CANSparkBase pivot;
    private final CANSparkBase feed;
    private final CANSparkBase flywheelTop;
    private final CANSparkBase flywheelBottom;
    private final RelativeEncoder pivotEncoder;
    private final RelativeEncoder feedEncoder;
    private final RelativeEncoder flywheelTopEncoder;
    private final RelativeEncoder flywheelBottomEncoder;
    private final DigitalInput beamBreak;

    public ShooterIOSparkMax(ShooterIOInputsAutoLogged input) {
        inputs = input;
        pivot = new CANSparkMax(Constants.Shooter.Ids.pivot, CANSparkLowLevel.MotorType.kBrushless);
        pivot.setIdleMode(CANSparkBase.IdleMode.kBrake);
        feed = new CANSparkMax(Constants.Shooter.Ids.feed, CANSparkLowLevel.MotorType.kBrushless);
        feed.setIdleMode(CANSparkBase.IdleMode.kBrake);
        flywheelTop = new CANSparkMax(Constants.Shooter.Ids.flywheelTop, CANSparkLowLevel.MotorType.kBrushless);
        flywheelBottom = new CANSparkMax(Constants.Shooter.Ids.flywheelBottom, CANSparkLowLevel.MotorType.kBrushless);
        pivotEncoder = pivot.getEncoder();
        feedEncoder = feed.getEncoder();
        flywheelTopEncoder = flywheelTop.getEncoder();
        flywheelBottomEncoder = flywheelBottom.getEncoder();
        flywheelTopEncoder.setVelocityConversionFactor(1);
        flywheelBottomEncoder.setVelocityConversionFactor(1);
        pivotEncoder.setPositionConversionFactor(Constants.Shooter.GearRatios.pivot * 360);
        pivotEncoder.setPosition(0);
        beamBreak = new DigitalInput(Constants.Shooter.Ids.beamBreak);
        pivot.setSmartCurrentLimit(38);
        feed.setSmartCurrentLimit(38);
        flywheelTop.setSmartCurrentLimit(38);
        flywheelBottom.setSmartCurrentLimit(38);
    }

    @Override
    public void updateInputs() {
        inputs.pivotPosition = pivotEncoder.getPosition();
        inputs.pivotVelocity = pivotEncoder.getVelocity();
        inputs.feedPosition = feedEncoder.getPosition();
        inputs.feedVelocity = feedEncoder.getVelocity();
        inputs.flywheelPositionTop = flywheelTopEncoder.getPosition();
        inputs.flywheelVelocityTop = flywheelTopEncoder.getVelocity();
        inputs.flywheelPositionBottom = flywheelBottomEncoder.getPosition();
        inputs.flywheelVelocityBottom = flywheelBottomEncoder.getVelocity();
        inputs.beamBreak = !beamBreak.get();
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
    public void setFlywheelVolts(double volts) {
        inputs.voltsAppliedTop = volts;
        flywheelTop.setVoltage(paralyzedFlywheel ? 0 : -MathUtil.clamp(volts, -12.0, 12.0));
        inputs.voltsAppliedBottom = -volts;
        flywheelBottom.setVoltage(paralyzedFlywheel ? 0 : -MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setFlywheelBrake(boolean brake) {
        inputs.brakeFlywheel = brake;
        flywheelTop.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
        flywheelBottom.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }
}
