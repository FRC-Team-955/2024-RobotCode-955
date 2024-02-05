package frc.robot.subsystem.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Constants;

public class IntakeIOSparkMax extends IntakeIO {

    private final CANSparkBase deploy;
    private final CANSparkBase intake;

    private final Ultrasonic ultrasonic;

    public IntakeIOSparkMax(IntakeIOInputsAutoLogged input) {
        inputs = input;
        deploy = new CANSparkMax(0, CANSparkBase.MotorType.kBrushless);
        intake = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushed);
        ultrasonic = new Ultrasonic(0, 0);
    }

    @Override
    public void updateInputs() {
        inputs.position = deploy.getEncoder().getPosition();
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
