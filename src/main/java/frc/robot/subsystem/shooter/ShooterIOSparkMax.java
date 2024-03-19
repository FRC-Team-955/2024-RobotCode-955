package frc.robot.subsystem.shooter;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

    private final SparkPIDController pivotController;
    private final SparkPIDController feedController;
    private final SparkPIDController flywheelTopController;
    private final SparkPIDController flywheelBottomController;

    private final DutyCycleEncoder pivotEncoderAbsolute;

    private final DigitalInput beamBreak;

    public ShooterIOSparkMax(ShooterIOInputs input) {
        inputs = input;

        pivot = new CANSparkMax(Constants.Shooter.Ids.pivot, CANSparkLowLevel.MotorType.kBrushless);
        feed = new CANSparkMax(Constants.Shooter.Ids.feed, CANSparkLowLevel.MotorType.kBrushless);
        flywheelTop = new CANSparkMax(Constants.Shooter.Ids.flywheelTop, CANSparkLowLevel.MotorType.kBrushless);
        flywheelBottom = new CANSparkMax(Constants.Shooter.Ids.flywheelBottom, CANSparkLowLevel.MotorType.kBrushless);

        pivot.restoreFactoryDefaults();
        feed.restoreFactoryDefaults();
        flywheelTop.restoreFactoryDefaults();
        flywheelBottom.restoreFactoryDefaults();

        pivot.setSmartCurrentLimit(40);
        feed.setSmartCurrentLimit(40);
        flywheelTop.setSmartCurrentLimit(40);
        flywheelBottom.setSmartCurrentLimit(40);

        pivot.setInverted(false);
        feed.setInverted(false);
        flywheelTop.setInverted(false);
        flywheelBottom.setInverted(true);

        pivotEncoder = pivot.getEncoder();
        feedEncoder = feed.getEncoder();
        flywheelTopEncoder = flywheelTop.getEncoder();
        flywheelBottomEncoder = flywheelBottom.getEncoder();

        pivotEncoderAbsolute = new DutyCycleEncoder(0);

        pivotEncoder.setPositionConversionFactor(Constants.Shooter.GearRatios.pivot * 360.0);
        pivotEncoder.setVelocityConversionFactor(Constants.Shooter.GearRatios.pivot * 360.0 * (1.0/60.0));
        feedEncoder.setPositionConversionFactor(Constants.Shooter.GearRatios.feed * 2.0 * Math.PI *
                Constants.Shooter.Dimensions.feedRadius);
        feedEncoder.setVelocityConversionFactor(Constants.Shooter.GearRatios.feed * 2.0 * Math.PI * (1.0/60.0) *
                Constants.Shooter.Dimensions.feedRadius);
        flywheelTopEncoder.setPositionConversionFactor(Constants.Shooter.GearRatios.flywheel * 2.0 * Math.PI *
                Constants.Shooter.Dimensions.flywheelRadius);
        flywheelTopEncoder.setVelocityConversionFactor(Constants.Shooter.GearRatios.flywheel * 2.0 * Math.PI *
                (1.0/60.0) * Constants.Shooter.Dimensions.flywheelRadius);
        flywheelBottomEncoder.setPositionConversionFactor(Constants.Shooter.GearRatios.flywheel * 2.0 * Math.PI *
                Constants.Shooter.Dimensions.flywheelRadius);
        flywheelBottomEncoder.setVelocityConversionFactor(Constants.Shooter.GearRatios.flywheel * 2.0 * Math.PI *
                (1.0/60.0) * Constants.Shooter.Dimensions.flywheelRadius);

        pivotEncoderAbsolute.setDistancePerRotation(360.0);

        pivot.setIdleMode(CANSparkBase.IdleMode.kBrake);
        feed.setIdleMode(CANSparkBase.IdleMode.kBrake);
        flywheelTop.setIdleMode(CANSparkBase.IdleMode.kCoast);
        flywheelBottom.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pivotController = pivot.getPIDController();
        feedController = feed.getPIDController();
        flywheelTopController = flywheelTop.getPIDController();
        flywheelBottomController = flywheelBottom.getPIDController();

        pivotController.setP(Constants.Shooter.Control.Pivot.kp, 0);
        pivotController.setI(Constants.Shooter.Control.Pivot.ki, 0);
        pivotController.setD(Constants.Shooter.Control.Pivot.kd, 0);

        feedController.setP(Constants.Shooter.Control.Feed.kp, 0);
        feedController.setI(Constants.Shooter.Control.Feed.ki, 0);
        feedController.setD(Constants.Shooter.Control.Feed.kd, 0);

        flywheelTopController.setP(Constants.Shooter.Control.Flywheel.kp, 0);
        flywheelTopController.setI(Constants.Shooter.Control.Flywheel.ki, 0);
        flywheelTopController.setD(Constants.Shooter.Control.Flywheel.kd, 0);
        flywheelBottomController.setP(Constants.Shooter.Control.Flywheel.kp, 0);
        flywheelBottomController.setI(Constants.Shooter.Control.Flywheel.ki, 0);
        flywheelBottomController.setD(Constants.Shooter.Control.Flywheel.kd, 0);

        pivotController.setReference(0, CANSparkBase.ControlType.kPosition);
        feedController.setReference(0, CANSparkBase.ControlType.kVelocity);
        flywheelTopController.setReference(0, CANSparkBase.ControlType.kVelocity);
        flywheelBottomController.setReference(0, CANSparkBase.ControlType.kVelocity);

        beamBreak = new DigitalInput(Constants.Shooter.Ids.beamBreak);

        zeroPivotRelative();
    }

    @Override
    public void updateSensors() {
        inputs.pivotPosition = pivotEncoder.getPosition();
        inputs.pivotVelocity = pivotEncoder.getVelocity();
        inputs.feedVelocity = feedEncoder.getVelocity();
        inputs.flywheelVelocityTop = flywheelTopEncoder.getVelocity();
        inputs.flywheelVelocityBottom = flywheelBottomEncoder.getVelocity();
        inputs.beamBreak = !beamBreak.get();
    }

    @Override
    public void updateApplications() {
        inputs.voltsAppliedPivot = pivot.getBusVoltage();
        inputs.ampsAppliedPivot = pivot.getOutputCurrent();
        inputs.voltsAppliedFeed = feed.getBusVoltage();
        inputs.ampsAppliedFeed = feed.getOutputCurrent();
        inputs.voltsAppliedFlywheelTop = flywheelTop.getBusVoltage();
        inputs.ampsAppliedFlywheelTop = flywheelTop.getOutputCurrent();
        inputs.voltsAppliedFlywheelBottom = flywheelBottom.getBusVoltage();
        inputs.ampsAppliedFlywheelBottom = flywheelBottom.getOutputCurrent();
    }

    @Override
    public void updatePivotController(double setpointDegrees, double feedforwardVolts) {
        pivotController.setReference(setpointDegrees, CANSparkBase.ControlType.kPosition, 0, feedforwardVolts);
    }

    @Override
    public void updateFeedController(double metersPerSecond, double feedforwardVolts) {
        feedController.setReference(metersPerSecond, CANSparkBase.ControlType.kVelocity, 0, feedforwardVolts);
    }

    @Override
    public void updateFlywheelController(double metersPerSecond, double feedforwardVoltsTop,
                                         double feedforwardVoltsBottom) {
        flywheelTopController.setReference(metersPerSecond, CANSparkBase.ControlType.kVelocity, 0,
                feedforwardVoltsTop);
        flywheelBottomController.setReference(metersPerSecond, CANSparkBase.ControlType.kVelocity, 0,
                feedforwardVoltsBottom);
    }

    @Override
    public void zeroPivotRelative() {
        pivotEncoder.setPosition(pivotEncoderAbsolute.getAbsolutePosition());
    }

    @Override
    public void zeroPivotAbsolute() {
        pivotEncoderAbsolute.reset();
    }
}
