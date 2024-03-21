package frc.robot.subsystem.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.conversion.AngleUtil;
import frc.robot.utility.object.ControlMode;
import org.littletonrobotics.junction.Logger;

/**
 * The Intake {@link Subsystem} for collecting notes from the ground
 */
public class Intake extends SubsystemBase {

    public static Intake instance;

    private final IntakeIO io;
    private final IntakeIOInputs inputs;

    private final ArmFeedforward deployFf;
    private final SimpleMotorFeedforward intakeFf;

    private final TrapezoidProfile deployMotionProfile;

    private ControlMode.Actuator deployControlMode = ControlMode.Actuator.Voltage;

    private double deploySetpoint = Constants.Intake.Setpoints.hover;
    public double intakeSetpoint = 0;

    private double deployVolts = 0;
    private double intakeVolts = 0;


    private Intake() {
        instance = this;
        inputs = new IntakeIOInputs();
        io = Robot.isSimulation() ? new IntakeIOSim(inputs) : new IntakeIOSparkMax(inputs);

        deployFf = new ArmFeedforward(Constants.Intake.Control.Deploy.ks, Constants.Intake.Control.Deploy.kg,
                Constants.Intake.Control.Deploy.kv, Constants.Intake.Control.Deploy.ka);
        intakeFf = new SimpleMotorFeedforward(Constants.Intake.Control.Intaking.ks,
                Constants.Intake.Control.Intaking.kv, Constants.Intake.Control.Intaking.ka);

        deployMotionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Constants.Intake.Control.Deploy.maxVelocity, Constants.Intake.Control.Deploy.maxAcceleration));
    }

    /**
     * Initialize the subsystem
     */
    public static void init() {
        if (instance == null) new Intake();
    }



    public void updateInputs() {
        io.updateSensors();
    }

    @Override
    public void periodic() {

        switch (deployControlMode) {
            case MotionProfile -> {
                TrapezoidProfile.State deployMotionSetpoint = deployMotionProfile.calculate(Constants.loopTime,
                        new TrapezoidProfile.State(inputs.deployPosition, inputs.deployVelocity),
                        new TrapezoidProfile.State(deploySetpoint, 0));
                io.setDeployController(deployMotionSetpoint.position, deployFf.calculate(
                        AngleUtil.degToRad(deployMotionSetpoint.position), AngleUtil.degToRad(deployMotionSetpoint.velocity)));
                io.setIntakeController(intakeSetpoint, intakeFf.calculate(inputs.intakeVelocity));
            }
            case ControlLoop -> {
                io.setDeployController(deploySetpoint, 0);
                io.setIntakeController(intakeSetpoint, 0);
            }
            case Voltage -> {
//                io.setDeployVolts(deployVolts);
                io.setIntakeVolts(intakeVolts);
            }
        }

        io.updateApplications();
        inputs.deployPositionSetpoint = deploySetpoint;
        inputs.intakeVelocitySetpoint = intakeSetpoint;
        Logger.processInputs("Intake", inputs);
    }

    public static void setDeployPositionSetpoint(double degrees) {
        instance.setDeployPositionSetpointI(degrees);
    }
    private void setDeployPositionSetpointI(double degrees) {
        deploySetpoint = degrees;
    }

    public static void setIntakeVelocitySetpoint(double metersPerSecond) {
        instance.setIntakeVelocitySetpointI(metersPerSecond);
    }
    private void setIntakeVelocitySetpointI(double metersPerSecond) {
        intakeSetpoint = metersPerSecond;
    }


    /**
     * Gets the current position of the intake
     * @return The current position of the intake deploy pivot
     */
    public static double getDeployPosition() { return instance.getDeployPositionI(); }
    private double getDeployPositionI() { return inputs.deployPosition; }



    /**
     * Gets whether the deploy pivot is at the angular position setpoint
     * @return Whether the right flywheel is within {@value Constants.Intake#deployTolerance} degrees from the setpoint
     */
    public static boolean atDeploySetpoint() {
        return instance.atDeploySetpointI(Constants.Intake.deployTolerance);
    }
    /**
     * Gets whether the deploy pivot is at the angular position setpoint within a given tolerance
     * @param tolerance The setpoint error tolerance in degrees
     * @return Whether the right flywheel is within the given tolerance from the setpoint
     */
    public static boolean atDeploySetpoint(double tolerance) {
        return instance.atDeploySetpointI(tolerance);
    }
    private boolean atDeploySetpointI(double tolerance) {
        return Math.abs(deploySetpoint - inputs.deployPosition) <= tolerance;
    }

    /**
     * Gets whether the intake speed is at the angular velocity setpoint
     * @return Whether intake speed is within {@value Constants.Intake#velocityTolerance} from the setpoint
     */
    public static boolean atIntakeSetpoint() {
        return instance.atIntakeSetpointI(Constants.Intake.velocityTolerance);
    }
    /**
     * Gets whether the intake speed is at the angular velocity setpoint within a given tolerance
     * @param tolerance The setpoint error tolerance in RPM
     * @return Whether intake speed is within the given tolerance from the setpoint
     */
    public static boolean atIntakeSetpoint(double tolerance) {
        return instance.atIntakeSetpointI(tolerance);
    }
    private boolean atIntakeSetpointI(double tolerance) {
        return Math.abs(intakeSetpoint - inputs.intakeVelocity) <= tolerance;
    }

    /**
     * Gets whether a note had been grabbed by the intake
     * @return Whether there is a note in the intake
     */
    public static boolean hasNote() {
        return instance.hasNoteI();
    }
    private boolean hasNoteI() { return inputs.limitSwitch; }

    public void setTuningMode(ControlMode.Tuning mode) {
        if (!DriverStation.isTestEnabled())
            return;

        switch (mode) {
            case Operation -> {
                deployControlMode = ControlMode.Actuator.MotionProfile;
            }
            case ControlLoop -> {
                deployControlMode = ControlMode.Actuator.ControlLoop;
            }
            case Characterization -> {
                deployControlMode = ControlMode.Actuator.Voltage;
            }
        }
    }

    public final class Characterization {
        public static SysIdRoutine deploy() {
            return new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(volts -> {
                System.out.println(volts.in(Units.Volts));
//                instance.deployVolts = volts.in(Units.Volts);
                instance.io.setDeployVolts(volts.in(Units.Volts));
            }, log -> {
                log.motor("Deploy").voltage(Units.Volts.of(instance.inputs.voltsAppliedDeploy))
                        .current(Units.Amps.of(instance.inputs.ampsAppliedDeploy))
                        .angularPosition(Units.Degrees.of(instance.inputs.deployPosition +
                                Constants.Intake.Control.Deploy.comAngleCompensation))
                        .angularVelocity(Units.DegreesPerSecond.of(instance.inputs.deployVelocity));
            }, Intake.instance, "Intake"));
        }
    }
}