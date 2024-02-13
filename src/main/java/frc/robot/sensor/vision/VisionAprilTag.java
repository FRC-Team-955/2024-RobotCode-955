package frc.robot.sensor.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.sensor.pose.Odometry;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

public class VisionAprilTag {
    private static final PhotonCamera distance = new PhotonCamera("distance");
    private static final PhotonCamera vertical = new PhotonCamera("vertical");

    private static PhotonPoseEstimator esimatorDistance = new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, distance, Constants.Vision.distancePosition);
    private static PhotonPoseEstimator esimatorVertical = new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, vertical, Constants.Vision.verticalPosition);

//    public static void update() {
//        Optional<EstimatedRobotPose> dist = esimatorDistance.update();
//        if (dist.isPresent())
//            Odometry.updateEstimateVision(Pose2d.);
//    }
}
