package frc.robot.subsystem.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.AngleUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveMod extends SubsystemBase {

    public static final SwerveMod[] instance = new SwerveMod[]{
            new SwerveMod(0), new SwerveMod(1), new SwerveMod(2),  new SwerveMod(3)
    };

    public enum IdleMode {
        Free,
        Brake
    }

    /**
     * The numerical id of the swerve module
     * <p> 0 = Front Left <p> 1 = Front Right <p> 2 = Back Right <p> 3 = Back Left
     */
    public final int modId;

    public final double localizedOffset;

    private final SwerveModIO io;
    private final SwerveModIOInputsAutoLogged inputs;

    private SwerveModuleState targetState;

    private final PIDController drivePid;
    private final PIDController anglePid;

    private SwerveModulePosition positionLast = new SwerveModulePosition();



    private SwerveMod(int id) {
        modId = id;

        localizedOffset = AngleUtil.unsignedRangeDegrees(-90.0 * id);
//        localizedOffset = 0.0;

        inputs = new SwerveModIOInputsAutoLogged();
        io = Robot.isSimulation() ? new SwerveModIOSim(inputs, id) : new SwerveModIOSpark(inputs, id);

        drivePid = new PIDController(5, 0, 0);
        anglePid = new PIDController(0.05, 0, 0.00001);
    }



    /**
     * Set the target state of the swerve module relative to the robot
     * @param state The {@link SwerveModuleState} object for the current desired state of the swerve module
     */
    public void setTargetState(SwerveModuleState state) {
        targetState = state;
//        targetState.angle = Rotation2d.fromDegrees(AngleUtil.unsignedRangeDegrees(state.angle.getDegrees() + localizedOffset));
    }

    /**
     * Set the target state of the swerve module relative to itself
     * @param state The {@link SwerveModuleState} object for the current desired state of the swerve module
     */
    public void setTargetStateLocalized(SwerveModuleState state) {
        targetState = state;
    }


    public void setIdleMode(IdleMode mode) {
        io.setIdleMode(mode);
    }

    @Override
    public void periodic() {
        positionLast = getCurrentPosition();

        io.updateInputs();
        Logger.processInputs("Mod " + modId, inputs);

        SwerveModuleState target = SwerveModuleState.optimize(targetState, getAnglePositionLocalizedRotation2d());
//        SwerveModuleState target = targetState;

        double av = MathUtil.clamp(anglePid.calculate(getAnglePositionLocalized(), AngleUtil.getUnwrappedSetpoint(getAnglePositionLocalized(), target.angle.getDegrees())), -12.0, 12.0);
        io.setAngleVolts(av);
        target.speedMetersPerSecond *= Math.cos(AngleUtil.degToRad(AngleUtil.signedRangeDifferenceDegrees(target.angle.getDegrees(), getAnglePositionLocalized())));
        double dv = MathUtil.clamp(drivePid.calculate(getDriveVelocity(), target.speedMetersPerSecond), -12.0, 12.0);
        io.setDriveVolts(dv);

        Logger.recordOutput("Mod/" + modId + "/VoltsAngle", av);
        Logger.recordOutput("Mod/" + modId + "/VoltsDrive", dv);
//        Logger.recordOutput("Mod/" + modId + "/TargetState", new SwerveModuleState(target.speedMetersPerSecond,
//                Rotation2d.fromDegrees(AngleUtil.unsignedRangeDegrees(target.angle.getDegrees() - localizedOffset))));
        Logger.recordOutput("Mod/" + modId + "/TargetState", target);
        Logger.recordOutput("Mod/" + modId + "/CurrentState", getCurrentState());
        Logger.recordOutput("Mod/" + modId + "/AngleError", targetState.angle.getDegrees() - getAnglePositionLocalized());
    }

    /**
     * Get the target state of the swerve module
     * @return The {@link SwerveModuleState} object for the current desired state of the swerve module
     */
    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Get the current physical state of the swerve module
     * @return The {@link SwerveModuleState} object for the current physical state of the swerve module
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveVelocity(), getAnglePositionRotation2d());
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAnglePositionRotation2d());
    }

    public SwerveModulePosition getPositionDelta() {
        return new SwerveModulePosition(getDrivePosition() - positionLast.distanceMeters, getAnglePositionRotation2d());
    }

    /**
     * Get the angular velocity of the drive wheel
     * @return The angular velocity of the drive wheel in degrees per second
     */
    public double getDriveAngularVelocity() {
        return inputs.driveVelocityDegSec;
    }

    /**
     * Get the velocity of the drive wheel
     * @return The velocity of the drive wheel as if it was driving on a surface in meters per second
     */
    public double getDriveVelocity() {
        return 2 * Math.PI * (inputs.driveVelocityDegSec / 360) * Constants.Swerve.wheelRadius;
    }

    /**
     * Get the position of the drive wheel
     * @return The accumulated position of the drive wheel in meters
     */
    public double getDrivePosition() {
        return 2 * Math.PI * (inputs.drivePositionDeg / 360) * Constants.Swerve.wheelRadius;
    }

    /**
     * Get the angular position of swerve module
     * @return The angular position of the drive wheel heading relative to its clockwise position from the front of the robot in degrees from 0 to 360
     */
    public double getAnglePosition() {
        return getAnglePositionLocalized();
//        return AngleUtil.unsignedRangeDegrees(inputs.anglePositionDeg - localizedOffset);
    }

    /**
     * Get the localized angular position of swerve module
     * @return The angular position of the drive wheel heading relative to its clockwise position from the clockwise most face of the swerve module in degrees from 0 to 360
     */
    public double getAnglePositionLocalized() {
        return inputs.anglePositionDeg;
    }

    /**
     * Get the angular position of swerve module
     * @return The angular position of the drive wheel heading relative to its clockwise position from the front of the robot
     */
    public Rotation2d getAnglePositionRotation2d() {
        return getAnglePositionLocalizedRotation2d();
//        return Rotation2d.fromDegrees(getAnglePosition());
    }

    /**
     * Get the angular position of swerve module
     * @return The angular position of the drive wheel heading relative to its clockwise position from the clockwise most face of the swerve module
     */
    public Rotation2d getAnglePositionLocalizedRotation2d() {
        return Rotation2d.fromDegrees(getAnglePositionLocalized());
    }

    /**
     * Get the angular velocity of the swerve module
     * @return The angular velocity of the drive wheel heading relative to its clockwise position from the front of the robot in degrees per second
     */
    public double getAngleVelocity() {
        return inputs.angleVelocityDegSec;
    }

    /**
     * Get the persistent angular position of the swerve module
     * @return The persistent angular position of the drive wheel heading relative to its clockwise position from the front of the robot in degrees from 0 to 360
     */
    public double getAnglePositionAbsolute() {
        return AngleUtil.unsignedRangeDegrees(inputs.anglePositionAbsoluteDeg - localizedOffset);
    }

    /**
     * Get the persistent angular position of the swerve module
     * @return The persistent angular position of the drive wheel heading relative to its clockwise position from the front of the robot in degrees from 0 to 360
     */
    public double getAnglePositionLocalizedAbsolute() {
        return inputs.anglePositionAbsoluteDeg;
    }
}
