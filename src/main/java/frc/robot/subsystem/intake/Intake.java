package frc.robot.subsystem.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.conversion.AngleUtil;
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

    private double deploySetpoint = Constants.Intake.Setpoints.hover;
    public double intakeSetpoint = 0;


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
        TrapezoidProfile.State deployMotionSetpoint = deployMotionProfile.calculate(Constants.loopTime,
                new TrapezoidProfile.State(inputs.deployPosition, inputs.deployVelocity),
                new TrapezoidProfile.State(deploySetpoint, 0));
        io.setDeployController(deployMotionSetpoint.position, deployFf.calculate(
                AngleUtil.degToRad(deployMotionSetpoint.position), AngleUtil.degToRad(deployMotionSetpoint.velocity)));
        io.setIntakeController(intakeSetpoint, intakeFf.calculate(inputs.intakeVelocity));

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
     * @return Whether the right flywheel is within {@value Constants.Intake#tolerance} degrees from the setpoint
     */
    public static boolean atDeploySetpoint() {
        return instance.atDeploySetpointI(Constants.Intake.tolerance);
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
     * Gets whether a note had been grabbed by the intake
     * @return Whether there is a note in the intake
     */
    public static boolean hasNote() {
        return instance.hasNoteI();
    }
    private boolean hasNoteI() { return inputs.limitSwitch; }
}
