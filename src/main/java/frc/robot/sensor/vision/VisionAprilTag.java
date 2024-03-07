package frc.robot.sensor.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.sensor.pose.Odometry;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

public class VisionAprilTag {
    private static final PhotonCamera distance = new PhotonCamera("Arducam_OV2311_USB_Camera");
    private static double xyStdDev = 0;
    private static double rotStdDev = 0;
    private static double avgArea = 0;
//    private static final PhotonCamera vertical = new PhotonCamera("vertical");

    private static PhotonPoseEstimator esimatorDistance = new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, distance, Constants.Vision.distancePosition);
//    private static PhotonPoseEstimator esimatorVertical = new PhotonPoseEstimator(
//            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
//            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, vertical, Constants.Vision.verticalPosition);

    public static void update() {
        Optional<EstimatedRobotPose> dist = esimatorDistance.update();
        if (dist.isPresent()) {
//            if (avgArea > 0.8 || odometryDifference < 0.5) {
//                xyStdDev = 0.5;
//                rotStdDev = 5;
//            } else if (avgArea > 0.5 && odometryDifference < 1) {
//                xyStdDev = 1;
//                rotStdDev = 8;
//            } else if (avgArea > 0.2 && odometryDifference < 2) {
//                xyStdDev = 2;
//                rotStdDev = 15;
//            } else if (avgArea > 0.05 && odometryDifference < 5) {
//                xyStdDev = 5;
//                rotStdDev = 15;
//            } else return;
//
//            if (tagCount >= 2) {
//                xyStdDev -= avgArea > 0.4 ? 0.125 : 0.25;
//                rotStdDev -= 4;
//            }
            xyStdDev = 2;
            rotStdDev = 75;
            Odometry.updateEstimateVision(dist.get().estimatedPose.toPose2d(), dist.get().timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(rotStdDev)));
        }
    }
}
