package frc.robot.subsystem.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

import java.lang.module.Configuration;

public class ClimberIOSparkMax extends ClimberIO {

    private final CANSparkBase left;
    private final CANSparkBase right;

    private final RelativeEncoder encoderLeft;
    private final RelativeEncoder encoderRight;

    public ClimberIOSparkMax(ClimberIOValuesAutoLogged input) {
        inputs = input;
        left = new CANSparkMax(Constants.Climber.leftId, CANSparkLowLevel.MotorType.kBrushless);
        right = new CANSparkMax(Constants.Climber.rightId, CANSparkLowLevel.MotorType.kBrushless);
        encoderLeft = left.getEncoder();
        encoderRight = right.getEncoder();

        // TODO relative encoder unit conversion
        // Conversion factors will be set to convert directly to meters of extension length
        encoderLeft.setPositionConversionFactor(Constants.Climber.gearRatio * 2 * Math.PI * Constants.Climber.radius);
        encoderLeft.setVelocityConversionFactor(Constants.Climber.gearRatio * 2 * Math.PI * Constants.Climber.radius);
        encoderRight.setPositionConversionFactor(Constants.Climber.gearRatio * 2 * Math.PI * Constants.Climber.radius);
        encoderRight.setVelocityConversionFactor(Constants.Climber.gearRatio * 2 * Math.PI * Constants.Climber.radius);
    }

    @Override
    public void updateInputs() {
        inputs.extentionPositionLeft = encoderLeft.getPosition();
        inputs.extentionVelocityLeft = encoderLeft.getVelocity();
        inputs.extentionPositionRight = encoderRight.getPosition();
        inputs.extentionVelocityRight = encoderRight.getVelocity();
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
