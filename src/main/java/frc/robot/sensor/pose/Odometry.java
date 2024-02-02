package frc.robot.sensor.pose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.SwerveMod;
import org.littletonrobotics.junction.Logger;

public class Odometry {

    private static final SwerveDrivePoseEstimator estimator;

    private static ChassisSpeeds last;
    private static ChassisSpeeds lastRelative;
    private static final Timer timer;

    public static Field2d loggedField = new Field2d();
    private static Pose2d targetPose = new Pose2d();
    private static Pose2d[] targetPath = new Pose2d[0];

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

    public static void updateEstimatePositions(SwerveModulePosition[] positions) {
        estimator.update(Gyro.getHeading(), Swerve.instance.getPositions());
    }

    public static void updateEstimateVision(Pose2d pose, double delay, Matrix<N3, N1> stdDevs) {
        estimator.addVisionMeasurement(pose, timer.get() - (delay / 1000), stdDevs);
    }

    public static void resetPose(Pose2d pose) {
        estimator.resetPosition(Gyro.getHeading(), Swerve.instance.getPositions(), pose);
    }

    public static void setSpeeds(ChassisSpeeds fieldRel, ChassisSpeeds robotRel) {
        last = fieldRel;
        lastRelative = robotRel;
    }

    public static void setTargetPose(Pose2d target) {
        targetPose = target;
    }

    public static void setTargetPath (Pose2d[] path) {
        targetPath = path;
    }

    public static void log() {
       loggedField.setRobotPose(getPose());
       loggedField.getObject("targetPose").setPose(targetPose);
       loggedField.getObject("path").setPoses(targetPath);
       Logger.recordOutput("Odom/Pose", estimator.getEstimatedPosition());
       Logger.recordOutput("Gyro", Gyro.getHeading());
    }

    public static Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public static ChassisSpeeds getSpeeds() { return last; }
    public static ChassisSpeeds getSpeedsRelative() { return lastRelative; }
}