package frc.robot.subsystem.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.AngleUtil;

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

    private final PIDController drivePid;
    private final PIDController anglePid;



    private SwerveMod(int id) {
        modId = id;

        inputs = new SwerveModIOInputsAutoLogged();
        io = Robot.isSimulation() ? new SwerveModIOSim(inputs, id) : new SwerveModIOSparkMax(inputs, id);

        drivePid = new PIDController(0, 0, 0);
        anglePid = new PIDController(0, 0, 0);
    }



    /**
     * Set the target state of the swerve module
     * @param state The {@link SwerveModuleState} object for the current desired state of the swerve module
     */
    public void setTargetState(SwerveModuleState state) {
        targetState = state;
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



    @Override
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
