package frc.robot.subsystem.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class IntakeIOSparkMax extends IntakeIO {

    public IntakeIOSparkMax(IntakeIOInputsAutoLogged input) {
        inputs = input;
        deploy = new CANSparkMax(0, CANSparkBase.MotorType.kBrushless);
        intake = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushed);
    }

    private final IntakeIOInputsAutoLogged inputs;

    private final CANSparkBase deploy;
    private final CANSparkBase intake;

    @Override
    public void updateInputs() {
        inputs.position = deploy.getEncoder().getPosition();

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
