package frc.robot.subsystem.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.AngleUtil;
import frc.robot.utility.InputUtil;
import org.littletonrobotics.junction.Logger;

public class SwerveMod extends SubsystemBase {

    public static final SwerveMod[] instance = new SwerveMod[]{
            new SwerveMod(0), new SwerveMod(1), new SwerveMod(2),  new SwerveMod(3)
    };

    /**
     * The numerical id of the swerve module
     * <p> 0 = Front Left <p> 1 = Front Right <p> 2 = Back Right <p> 3 = Back Left
     */
    public final int modId;

    private final SwerveModIO io;
    private final SwerveModIOInputsAutoLogged inputs;

    private SwerveModuleState targetState;

    private final PIDController anglePid;

    private SwerveModulePosition positionLast = new SwerveModulePosition();



    private SwerveMod(int id) {
        modId = id;

        inputs = new SwerveModIOInputsAutoLogged();
        io = Robot.isSimulation() ? new SwerveModIOSim(inputs, id) : new SwerveModIOSpark(inputs, id);

        anglePid = new PIDController(0.1, 0, 0.00001);

        io.syncEncoders();
    }



    @Override
    public void periodic() {
        positionLast = getCurrentPosition();

        io.updateInputs();
        Logger.processInputs("Swerve/Mod" + modId, inputs);

        SwerveModuleState target = SwerveModuleState.optimize(targetState, getAnglePositionRotation2d());

        double av = MathUtil.clamp(anglePid.calculate(getAnglePosition(),
                AngleUtil.getUnwrappedSetpoint(getAnglePosition(), target.angle.getDegrees())), -12.0, 12.0);
        io.setAngleVolts(av);
        target.speedMetersPerSecond *= Math.cos(AngleUtil.degToRad(
                AngleUtil.signedRangeDifferenceDegrees(target.angle.getDegrees(), getAnglePosition())));
        double dv = (target.speedMetersPerSecond / Constants.Swerve.maxFreeSpeed) * 12;
        io.setDriveVolts(dv);

        Logger.recordOutput("Swerve/Mod" + modId + "/VoltsAngle", av);
        Logger.recordOutput("Swerve/Mod" + modId + "/VoltsDrive", dv);
        Logger.recordOutput("Swerve/Mod" + modId + "/TargetState", target);
        Logger.recordOutput("Swerve/Mod" + modId + "/CurrentState", getCurrentState());
        Logger.recordOutput("Swerve/Mod" + modId + "/AngleError", targetState.angle.getDegrees() - getAnglePosition());
    }



    /**
     * Sets the target state of the swerve module relative to the robot
     * @param state The {@link SwerveModuleState} object for the current desired state of the swerve module
     */
    public void setTargetState(SwerveModuleState state) {
        targetState = state;
        targetState.angle = Rotation2d.fromDegrees(AngleUtil.unsignedRangeDegrees(state.angle.getDegrees()));
    }

    /**
     * Sets the target state of the swerve module relative to itself
     * @param state The {@link SwerveModuleState} object for the current desired state of the swerve module
     */
    public void setTargetStateLocalized(SwerveModuleState state) {
        // TODO THIS
        targetState = state;
    }

    /**
     * Syncs the relative encoder on the angle motor with the absolute encoder on the module
     */
    public void syncEncoders() {
        io.syncEncoders();
    }

    /**
     * Sets whether the module should be in brake mode
     * @param brake Whether the angle and drive motors should be in brake mode
     */
    public void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }



    /**
     * Gets the target state of the swerve module
     * @return The {@link SwerveModuleState} object for the current desired state of the swerve module
     */
    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Gets the current physical state of the swerve module
     * @return The {@link SwerveModuleState} object for the current physical state of the swerve module
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveVelocity(), getAnglePositionRotation2d());
    }

    /**
     * Gets the current position of the swerve module
     * @return The {@link SwerveModulePosition} representing the accumulated drive wheel rotation and
     * the current angle heading
     */
    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAnglePositionRotation2d());
    }

    /**
     * Gets the change in drive position since the last tick
     * @return The {@link SwerveModulePosition} representing the change in drive position since last tick along
     * with the current module rotation
     */
    public SwerveModulePosition getPositionDelta() {
        return new SwerveModulePosition(getDrivePosition() - positionLast.distanceMeters,
                getAnglePositionRotation2d());
    }

    /**
     * Gets the angular velocity of the drive wheel
     * @return The angular velocity of the drive wheel in degrees per second
     */
    public double getDriveAngularVelocity() {
        return inputs.driveVelocityDegSec;
    }

    /**
     * Gets the velocity of the drive wheel
     * @return The velocity of the drive wheel as if it was driving on a surface in meters per second
     */
    public double getDriveVelocity() {
        return 2 * Math.PI * (inputs.driveVelocityDegSec / 360) * Constants.Swerve.wheelRadius;
    }

    /**
     * Gets the position of the drive wheel
     * @return The accumulated position of the drive wheel in meters
     */
    public double getDrivePosition() {
        return 2 * Math.PI * (inputs.drivePositionDeg / 360) * Constants.Swerve.wheelRadius;
    }

    /**
     * Gets the angular position of swerve module
     * @return The angular position of the drive wheel heading relative to its clockwise position from
     * the front of the robot in degrees from 0 to 360
     */
    public double getAnglePosition() {
            return inputs.anglePositionDeg;
    }

    /**
     * Gets the angular position of swerve module
     * @return The angular position of the drive wheel heading relative to its clockwise position from
     * the front of the robot
     */
    public Rotation2d getAnglePositionRotation2d() {
        return Rotation2d.fromDegrees(getAnglePosition());
    }

    /**
     * Gets the angular velocity of the swerve module
     * @return The angular velocity of the drive wheel heading relative to its clockwise position from
     * the front of the robot in degrees per second
     */
    public double getAngleVelocity() {
        return inputs.angleVelocityDegSec;
    }

    /**
     * Gets the persistent angular position of the swerve module
     * @return The persistent angular position of the drive wheel heading relative to its clockwise position from
     * the front of the robot in degrees from 0 to 360
     */
    public double getAnglePositionAbsolute() {
        return AngleUtil.unsignedRangeDegrees(inputs.anglePositionAbsoluteDeg);
    }
}
