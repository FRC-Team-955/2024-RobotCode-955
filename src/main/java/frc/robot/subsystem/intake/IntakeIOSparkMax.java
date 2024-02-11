package frc.robot.subsystem.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Constants;

public class IntakeIOSparkMax extends IntakeIO {

    private final CANSparkBase deploy;
    private final CANSparkBase intake;

    private final RelativeEncoder deployEncoder;

    private final Ultrasonic ultrasonic;

    public IntakeIOSparkMax(IntakeIOInputsAutoLogged input) {
        inputs = input;
        deploy = new CANSparkMax(Constants.Intake.deployId, CANSparkBase.MotorType.kBrushless);
        intake = new CANSparkMax(Constants.Intake.intakeId, CANSparkLowLevel.MotorType.kBrushed);
        deployEncoder = deploy.getEncoder();
        deployEncoder.setPositionConversionFactor(Constants.Intake.gearRatioDeploy);
        deployEncoder.setVelocityConversionFactor(Constants.Intake.gearRatioDeploy);
        ultrasonic = new Ultrasonic(Constants.Intake.pingId, Constants.Intake.echoId);
    }

    @Override
    public void updateInputs() {
        inputs.position = deployEncoder.getPosition();
        inputs.velocity = deployEncoder.getVelocity();
        inputs.noteCaptured = ultrasonic.getRangeInches() <= Constants.Intake.UltrasonicRanges.noteCaptureDistance;
        inputs.noteSecured = ultrasonic.getRangeInches() <= Constants.Intake.UltrasonicRanges.noteSecureDistance;
    }

    @Override
    public void setDeployMotor(double volts) {
        deploy.setVoltage(volts);
    }

    @Override
    public void setIntakeMotor(double volts) {
        intake.setVoltage(volts);
    }

    @Override
    public void setDeployBrake(boolean brake) {
        deploy.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }
}
