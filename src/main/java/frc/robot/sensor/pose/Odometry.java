package frc.robot.sensor.pose;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.SwerveMod;
import frc.robot.utility.conversion.AngleUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Manages the positional estimation of the robot
 */
public class Odometry {

    private static final SwerveDrivePoseEstimator estimator;
    private static ChassisSpeeds fieldRel;
    private static final Timer timer;



    static {
        estimator = new SwerveDrivePoseEstimator(new SwerveDriveKinematics(Constants.Swerve.modulePositions),
                Gyro.getHeading(), new SwerveModulePosition[] {
                new SwerveModulePosition(0, SwerveMod.instance[0].getAnglePositionRotation2d()),
                new SwerveModulePosition(0, SwerveMod.instance[1].getAnglePositionRotation2d()),
                new SwerveModulePosition(0, SwerveMod.instance[2].getAnglePositionRotation2d()),
                new SwerveModulePosition(0, SwerveMod.instance[3].getAnglePositionRotation2d()),
        }, new Pose2d());
        timer = new Timer();
        timer.start();
    }



    /**
     * Updates the odometry position estimate. Called periodically by the {@link Swerve} {@link Subsystem}
     */
    public static void updateEstimatePositions() {
        estimator.update(Rotation2d.fromDegrees(AngleUtil.unsignedRangeDegrees(Gyro.getHeading().getDegrees() + 0)), Swerve.getPositions());
    }

    /**
     * Updates the current position estimate with vision information
     * @param pose The estimated {@link Pose2d} from the vision frame
     * @param timestamp The timestamp the frame was captured at
     * @param stdDevs The standard deviations of the vision estimate
     */
    public static void updateEstimateVision(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        estimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }
    public static void updateEstimateVision(Pose2d pose, double timestamp) {
        estimator.addVisionMeasurement(pose, timestamp);
    }

    /**
     * Resets the estimate of the robot pose
     * @param pose The pose to reset the estimate to
     */
    public static void resetPose(Pose2d pose) {
        estimator.resetPosition(Gyro.getHeading(), Swerve.getPositions(), pose);
    }

    public static void updateChassisSpeedsEstimate(ChassisSpeeds estimate) {
        fieldRel = estimate;
    }



    /**
     * Get the current robot pose
     * @return A {@link Pose2d} representing the current estimated robot pose
     */
    public static Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public static Translation2d getVelocity() {
        return new Translation2d(fieldRel.vxMetersPerSecond, fieldRel.vyMetersPerSecond);
    }



    /**
     * Logs the current pose and {@link Gyro} estimates for the robot. Called periodically in {@link frc.robot.Robot}
     */
    public static void log() {
       Logger.recordOutput("Odom/Pose", estimator.getEstimatedPosition());
       Logger.recordOutput("Gyro", Gyro.getHeading());
    }
}