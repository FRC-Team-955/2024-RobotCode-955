package frc.lib.subsystems.arm;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim extends ArmIO {
    private SingleJointedArmSim sim;
    private PIDController pid;

    private double appliedVolts;
    private boolean closedLoop = false;
    private double ffVolts;

    private final DCMotor motor;
    private final double jKgMetersSquared;
    private final double armLength;

    public ArmIOSim(DCMotor motor, double armLength) {
        this(motor, 0.0001, armLength);
    }

    public ArmIOSim(DCMotor motor, double jKgMetersSquared, double armLength) {
        this.motor = motor;
        this.jKgMetersSquared = jKgMetersSquared;
        this.armLength = armLength;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = pid.calculate(sim.getAngleRads()) + ffVolts;
            sim.setInputVoltage(appliedVolts);
        }

        inputs.positionRad = sim.getAngleRads();
        inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        sim.setInputVoltage(appliedVolts);
        closedLoop = false;
    }

    @Override
    public void setSetpoint(double setpointPositionRad, double ffVolts) {
        this.ffVolts = ffVolts;
        closedLoop = true;
    }

    @Override
    public void setPosition(double currentPositionRad) {
        sim.setState(currentPositionRad, 0);
    }

    @Override
    public void stop() {
        sim.setInputVoltage(0);
    }

    @Override
    public void configurePID(PIDConstants pidConstants) {
        pid = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
        pid.setIZone(pidConstants.iZone);
    }

    @Override
    public void setGearRatio(double gearRatio) {
        sim = new SingleJointedArmSim(
                motor,
                gearRatio,
                jKgMetersSquared,
                armLength,
                0.0,
                0.0,
                true,
                0.0
        );
    }
}
