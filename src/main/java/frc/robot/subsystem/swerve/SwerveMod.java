package frc.robot.subsystem.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.AngleUtil;

public class SwerveMod{

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



    private SwerveMod(int id) {
        modId = id;

        localizedOffset = AngleUtil.unsignedRangeDegrees(-90.0 * id);

        inputs = new SwerveModIOInputsAutoLogged();
        io = Robot.isSimulation() ? new SwerveModIOSim(inputs, id) : new SwerveModIOSpark(inputs, id);

        drivePid = new PIDController(0.1, 0, 0);
        anglePid = new PIDController(0.1, 0, 0);
    }



    /**
     * Set the target state of the swerve module relative to the robot
     * @param state The {@link SwerveModuleState} object for the current desired state of the swerve module
     */
    public void setTargetState(SwerveModuleState state) {
        state.angle = Rotation2d.fromDegrees(AngleUtil.unsignedRangeDegrees(state.angle.getDegrees() + localizedOffset));
        targetState = state;
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
        return new SwerveModuleState();
    }



    public void updateInputs() {
        io.updateInputs();
    }

    public void periodic() {
        SwerveModuleState target = SwerveModuleState.optimize(targetState, new Rotation2d(inputs.drivePositionDeg));

        io.setAngleVolts(anglePid.calculate(0, AngleUtil.signedRangeDifferenceDegrees(inputs.anglePositionDeg, target.angle.getDegrees())));
        io.setDriveVolts(drivePid.calculate(getDriveVelocity(), target.speedMetersPerSecond));
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
     * Get the angular position of swerve module
     * @return The angular position of the drive wheel heading relative to its clockwise position from the front of the robot in degrees from 0 to 360
     */
    public double getAnglePosition() {
        return inputs.anglePositionDeg;
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
        return inputs.anglePositionAbsoluteDeg;
    }
}