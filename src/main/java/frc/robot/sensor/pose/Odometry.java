package frc.robot.sensor.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.ObjectUtil;

import java.util.PriorityQueue;

public class Odometry {

    private static final PriorityQueue<ChassisSpeeds> log = new PriorityQueue<ChassisSpeeds>();

    private static ChassisSpeeds last;
    private static ChassisSpeeds lastRelative;

    private static Pose2d estimate;

    public static Field2d loggedField = new Field2d();
    private static Pose2d targetPose;
    private static Pose2d[] targetPath;

    public static void updateEstimateChassisSpeeds(ChassisSpeeds speeds, ChassisSpeeds speedsRelative) {
        lastRelative = speedsRelative;
        last = speeds;
        log.add(speeds);
        if (log.size() > Constants.Swerve.PoseEstimation.maxLogTicks) log.poll();
        estimate.transformBy(ObjectUtil.toTransform(speeds));
    }

    public static void updateEstimatePose(Pose2d pose, double delay) {
        if (delay / 20 > log.size()) return;

        estimate = pose;

        for (int i = 0; i < (delay / 20) - 1; i++) {
            log.poll();
        }
        estimate.transformBy(ObjectUtil.toTransform(log.poll()).times(1 - ((delay / 20) % 1)));
        for (int i = 0; i < log.size(); i++) {
            ChassisSpeeds speeds = log.poll();
            estimate.transformBy(ObjectUtil.toTransform(speeds));
            log.add(speeds);
        }
    }

    public static void updatePostEstimateTrustworthy(Pose2d pose) {
        estimate = pose;
        log.clear();
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
    }

    public static Pose2d getPose() {
        return new Pose2d(estimate.getTranslation(), Gyro.getHeading());
    }

    public static ChassisSpeeds getSpeeds() { return last; }
    public static ChassisSpeeds getSpeedsRelative() { return lastRelative; }
}