package frc.robot.subsystem.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class ClimberIOSparkMax extends ClimberIO {

    private final CANSparkBase left;
    private final CANSparkBase right;

    private final RelativeEncoder encoderLeft;
    private final RelativeEncoder encoderRight;

    public ClimberIOSparkMax(ClimberIOInputsAutoLogged input) {
        inputs = input;
        left = new CANSparkMax(Constants.Climber.leftId, CANSparkLowLevel.MotorType.kBrushless);
        right = new CANSparkMax(Constants.Climber.rightId, CANSparkLowLevel.MotorType.kBrushless);
        left.setIdleMode(CANSparkBase.IdleMode.kBrake);
        right.setIdleMode(CANSparkBase.IdleMode.kBrake);
        left.setInverted(true);
        right.setInverted(true);
        encoderLeft = left.getEncoder();
        encoderRight = right.getEncoder();

        encoderLeft.setPositionConversionFactor(1);
        encoderLeft.setVelocityConversionFactor(1);
        encoderRight.setPositionConversionFactor(1);
        encoderRight.setVelocityConversionFactor(1);

        left.setSmartCurrentLimit(50);
        right.setSmartCurrentLimit(50);
    }

    @Override
    public void updateInputs() {
        inputs.extensionPositionLeft = encoderLeft.getPosition();
        inputs.extensionVelocityLeft = encoderLeft.getVelocity();
        inputs.extensionPositionRight = encoderRight.getPosition();
        inputs.extensionVelocityRight = encoderRight.getVelocity();
    }

    @Override
    public void setLeftVolts(double volts) {
        left.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setRightVolts(double volts) {
        right.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setLeftBrake(boolean brake) {
        left.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    @Override
    public void setRightBrake(boolean brake) {
        right.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }
}
