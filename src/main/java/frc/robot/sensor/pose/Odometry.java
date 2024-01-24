package frc.robot.sensor.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.ObjectUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.PriorityQueue;

public class Odometry {

    private static final PriorityQueue<Double> logX = new PriorityQueue<Double>();
    private static final PriorityQueue<Double> logY = new PriorityQueue<Double>();
    private static final PriorityQueue<Double> logR = new PriorityQueue<Double>();

    private static ChassisSpeeds last;
    private static ChassisSpeeds lastRelative;

    private static Pose2d estimate = new Pose2d();

    public static Field2d loggedField = new Field2d();
    private static Pose2d targetPose = new Pose2d();
    private static Pose2d[] targetPath = new Pose2d[0];

    public static void updateEstimateChassisSpeeds(ChassisSpeeds speeds, ChassisSpeeds speedsRelative) {
        lastRelative = speedsRelative;
        last = speeds;
        logX.add(speeds.vxMetersPerSecond);
        logY.add(speeds.vyMetersPerSecond);
        logR.add(speeds.omegaRadiansPerSecond);
        if (logX.size() > Constants.Swerve.PoseEstimation.maxLogTicks) {
            logX.poll();
            logY.poll();
            logR.poll();
        }
        estimate.transformBy(ObjectUtil.toTransform(speeds));
    }

    public static void updateEstimatePose(Pose2d pose, double delay) {
        if (delay / 20 > logX.size()) return;

        estimate = pose;

        for (int i = 0; i < (delay / 20) - 1; i++) {
            logX.poll();
            logY.poll();
            logR.poll();
        }
        estimate.transformBy(new Transform2d(logX.poll(), logY.poll(), Rotation2d.fromRadians(logR.poll())).times(1 - ((delay / 20) % 1)));
        for (int i = 0; i < logX.size(); i++) {
            double x = logX.poll();
            double y = logY.poll();
            double r = logR.poll();
            estimate.transformBy(new Transform2d(x, y, Rotation2d.fromRadians(r)));
            logX.add(x);
            logY.add(y);
            logR.add(r);
        }
    }

    public static void updatePostEstimateTrustworthy(Pose2d pose) {
        estimate = pose;
        logX.clear();
        logY.clear();
        logR.clear();
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
       Logger.recordOutput("Odom/Pose", getPose());
    }

    public static Pose2d getPose() {
        return new Pose2d(estimate.getTranslation(), Gyro.getHeading());
    }

    public static ChassisSpeeds getSpeeds() { return last; }
    public static ChassisSpeeds getSpeedsRelative() { return lastRelative; }
}