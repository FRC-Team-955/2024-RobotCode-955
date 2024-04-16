package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final SimpleMotorFeedforward ffModel;
    private final SysIdRoutine sysId;

    private static Flywheel instance;

    public static Flywheel get() {
        return instance;
    }

    public Flywheel(FlywheelIO io) {
        if (instance != null)
            throw new RuntimeException("Duplicate subsystem created!");
        instance = this;

        this.io = io;

        switch (Constants.mode) {
            case REAL, REPLAY -> {
                ffModel = new SimpleMotorFeedforward(0.1, 0.05);
                io.configurePID(1.0, 0.0, 0.0);
            }
            case SIM -> {
                ffModel = new SimpleMotorFeedforward(0.0, 0.03);
                io.configurePID(0.5, 0.0, 0.0);
            }
            default -> throw new RuntimeException("unreachable");
        }

        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Flywheel", inputs);
    }

    /**
     * Run open loop at the specified voltage.
     */
    public void runVolts(double volts) {
        io.setVoltage(volts);
    }

    /**
     * Run closed loop at the specified velocity.
     */
    public void runVelocity(double velocityRPM) {
        var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

        // Log flywheel setpoint
        Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
    }

    /**
     * Stops the flywheel.
     */
    public void stop() {
        io.stop();
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    /**
     * Returns the current velocity in RPM.
     */
    @AutoLogOutput
    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    }

    /**
     * Returns the current velocity in radians per second.
     */
    public double getCharacterizationVelocity() {
        return inputs.velocityRadPerSec;
    }
}
