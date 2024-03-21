package frc.robot.subsystem.intake;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.utility.object.MaxbotixUltrasonic;

public class IntakeIOSparkMax extends IntakeIO {

    public static boolean paralyzedDeploy = false;
    public static boolean paralyzedIntake = false;

    private final CANSparkBase deploy;
    private final CANSparkBase intake;

    private final RelativeEncoder deployEncoder;
    private final RelativeEncoder intakeEncoder;

    private final SparkPIDController deployController;
    private final SparkPIDController intakeController;

    private final AbsoluteEncoder deployEncoderAbsolute;

    private final DigitalInput limitSwitch;
    private final Debouncer limitSwitchDebouncer;

    public IntakeIOSparkMax(IntakeIOInputs input) {
        inputs = input;

        deploy = new CANSparkMax(Constants.Intake.deployId, CANSparkBase.MotorType.kBrushless);
        intake = new CANSparkMax(Constants.Intake.intakeId, CANSparkLowLevel.MotorType.kBrushless);

        deploy.restoreFactoryDefaults();
        intake.restoreFactoryDefaults();

        deploy.setSmartCurrentLimit(40);
        intake.setSmartCurrentLimit(40);

        deploy.setInverted(false);
        intake.setInverted(true);

        deployEncoder = deploy.getEncoder();
        intakeEncoder = intake.getEncoder();

        deployEncoderAbsolute = deploy.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        deployEncoder.setPositionConversionFactor(Constants.Intake.gearRatioDeploy * 360.0);
        deployEncoder.setVelocityConversionFactor(Constants.Intake.gearRatioDeploy * 360.0 * (1.0/60.0));
        intakeEncoder.setPositionConversionFactor(Constants.Intake.gearRatioIntake * 2.0 * Math.PI *
                Constants.Intake.Dimensions.intakeRadius);
        intakeEncoder.setVelocityConversionFactor(Constants.Intake.gearRatioIntake * 2.0 * Math.PI * (1.0/60.0) *
                Constants.Intake.Dimensions.intakeRadius);

        deployEncoderAbsolute.setPositionConversionFactor(360.0);
        deployEncoderAbsolute.setPositionConversionFactor(360.0 / 60);

        deploy.setIdleMode(CANSparkBase.IdleMode.kBrake);
        intake.setIdleMode(CANSparkBase.IdleMode.kBrake);

        deployController = deploy.getPIDController();
        intakeController = intake.getPIDController();

        deployController.setP(Constants.Intake.Control.Deploy.kp, 0);
        deployController.setI(Constants.Intake.Control.Deploy.ki, 0);
        deployController.setD(Constants.Intake.Control.Deploy.kd, 0);

        intakeController.setP(Constants.Intake.Control.Intaking.kp, 0);
        intakeController.setI(Constants.Intake.Control.Intaking.ki, 0);
        intakeController.setD(Constants.Intake.Control.Intaking.kd, 0);

        deployController.setReference(0, CANSparkBase.ControlType.kPosition);
        intakeController.setReference(0, CANSparkBase.ControlType.kVelocity);

        limitSwitch = new DigitalInput(Constants.Intake.limitSwitchId);
        limitSwitchDebouncer = new Debouncer(Constants.Intake.limitSwitchDenoiseTime, Debouncer.DebounceType.kBoth);

        zeroDeployRelative();
    }

    @Override
    public void updateSensors() {
        inputs.deployPosition = deployEncoder.getPosition();
        inputs.deployVelocity = deployEncoder.getVelocity();
        inputs.intakeVelocity = intakeEncoder.getVelocity();
        inputs.limitSwitch = limitSwitchDebouncer.calculate(limitSwitch.get());
    }

    @Override
    public void updateApplications() {
        inputs.voltsAppliedDeploy = deploy.getAppliedOutput() * deploy.getBusVoltage();
        inputs.ampsAppliedDeploy = deploy.getOutputCurrent();
        inputs.voltsAppliedIntake = intake.getAppliedOutput() * intake.getBusVoltage();
        inputs.ampsAppliedIntake = intake.getOutputCurrent();
    }

    @Override
    public void setDeployController(double setpointDegrees, double feedforwardVolts) {
        deployController.setReference(setpointDegrees, CANSparkBase.ControlType.kPosition, 0, feedforwardVolts);
    }

    @Override
    public void setIntakeController(double setpointMetersPerSecond, double feedforwardVolts) {
        intakeController.setReference(setpointMetersPerSecond, CANSparkBase.ControlType.kVelocity, 0,
                feedforwardVolts);
    }

    @Override
    public void setDeployVolts(double volts) {
        deploy.setVoltage(volts);
    }

    @Override
    public void setIntakeVolts(double volts) {
        intake.setVoltage(volts);
    }

    @Override
    public void zeroDeployRelative() {
        deployEncoder.setPosition(deployEncoderAbsolute.getPosition());
    }

    @Override
    public void zeroDeployAbsolute() {

//        deployEncoderAbsolute.setZeroOffset();
    }
}
