package frc.robot.subsystem.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;
import frc.robot.utility.object.MaxbotixUltrasonic;

public class IntakeIOSparkMax extends IntakeIO {

    public static boolean paralyzedDeploy = false;
    public static boolean paralyzedIntake = false;

    private final CANSparkBase deploy;
    private final CANSparkBase intake;

    private final RelativeEncoder deployEncoder;
//    private final RelativeEncoder intakeEncoder;

    private final MaxbotixUltrasonic ultrasonic;
    private final LinearFilter ultrasonicFilter;

    public IntakeIOSparkMax(IntakeIOInputsAutoLogged input) {
        inputs = input;
        deploy = new CANSparkMax(Constants.Intake.deployId, CANSparkBase.MotorType.kBrushless);
        intake = new CANSparkMax(Constants.Intake.intakeId, CANSparkLowLevel.MotorType.kBrushed);
        deploy.setInverted(true);
        intake.setInverted(true);
        deployEncoder = deploy.getEncoder();
//        intakeEncoder = intake.getEncoder();
        deployEncoder.setPositionConversionFactor(Constants.Intake.gearRatioDeploy * 360);
        deployEncoder.setVelocityConversionFactor(Constants.Intake.gearRatioDeploy * 360);
        ultrasonic = new MaxbotixUltrasonic(Constants.Intake.ultrasonicId);
        ultrasonicFilter = LinearFilter.movingAverage(3);
    }

    @Override
    public void updateInputs() {
        inputs.position = deployEncoder.getPosition();
        inputs.velocity = deployEncoder.getVelocity();
        inputs.ultrasonicRange = ultrasonicFilter.calculate(ultrasonic.getRangeInches());
//        inputs.intakeVelocity = intakeEncoder.getVelocity();
        inputs.intakeAmpDraw = intake.getOutputCurrent();
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
