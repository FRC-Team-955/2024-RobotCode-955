package frc.robot.subsystem.intake;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.utility.object.MaxbotixUltrasonic;

public class IntakeIOSparkMax extends IntakeIO {

    public static boolean paralyzedDeploy = false;
    public static boolean paralyzedIntake = false;

    private final CANSparkBase deploy;
    private final CANSparkBase intake;

    private final RelativeEncoder deployEncoder;
    private final SparkAbsoluteEncoder absolute;

    private final DigitalInput limitSwitch;

    public IntakeIOSparkMax(IntakeIOInputsAutoLogged input) {
        inputs = input;
        deploy = new CANSparkMax(Constants.Intake.deployId, CANSparkBase.MotorType.kBrushless);
        intake = new CANSparkMax(Constants.Intake.intakeId, CANSparkLowLevel.MotorType.kBrushless);
        deploy.setInverted(false);
        intake.setInverted(true);
        deployEncoder = deploy.getEncoder();
        absolute = deploy.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        deploy.setIdleMode(CANSparkBase.IdleMode.kCoast);
        deployEncoder.setPositionConversionFactor(Constants.Intake.gearRatioDeploy * 360);
        deployEncoder.setVelocityConversionFactor(Constants.Intake.gearRatioDeploy * 360);
        absolute.setPositionConversionFactor(Constants.Intake.gearRatioDeploy * 360);
        absolute.setVelocityConversionFactor(Constants.Intake.gearRatioDeploy * 360);
        deployEncoder.setPosition(Constants.Intake.Setpoints.start);
        limitSwitch = new DigitalInput(Constants.Intake.limitSwitchId);
        deploy.setSmartCurrentLimit(38);
        intake.setSmartCurrentLimit(30);
    }

    @Override
    public void updateInputs() {
        inputs.position = deployEncoder.getPosition();
        inputs.velocity = deployEncoder.getVelocity();
        inputs.limitSwitch = limitSwitch.get();
    }

    @Override
    public void setDeployMotor(double volts) {
        inputs.voltsAppliedDeploy = volts;
        deploy.setVoltage(paralyzedDeploy ? 0 : MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setIntakeMotor(double volts) {
        inputs.voltsAppliedIntake = volts;
        intake.setVoltage(paralyzedIntake ? 0 : MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setDeployBrake(boolean brake) {
        inputs.brakeDeploy = brake;
        deploy.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }
}
