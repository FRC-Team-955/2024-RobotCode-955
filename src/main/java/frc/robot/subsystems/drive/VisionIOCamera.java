package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class VisionIOCamera extends VisionIO {
    private static final Transform3d ROBOT_TO_CAM = new Transform3d(
            // Supposedly forward is positive x, left is positive y, up is positive z
            new Translation3d(Units.inchesToMeters(-10.325), Units.inchesToMeters(-5.135), Units.inchesToMeters(12.458)),
            new Rotation3d(0, -Units.degreesToRadians(30), Math.PI)
    );
    private static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final PhotonCamera cam;
    private final PhotonPoseEstimator photonPoseEstimator;

    // ids are Shooter_Cam
    public VisionIOCamera(String id) {
        cam = new PhotonCamera(id);
        photonPoseEstimator = new PhotonPoseEstimator(
                APRILTAG_FIELD_LAYOUT,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cam,
                ROBOT_TO_CAM
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        var result = cam.getLatestResult();
        inputs.hasTargets = result.hasTargets();
        Optional<EstimatedRobotPose> poseUpdate = photonPoseEstimator.update();
        inputs.hasEstimatedPose = poseUpdate.isPresent();
        List<PhotonTrackedTarget> targets = null;
        PhotonTrackedTarget bestTarget = null;
        if (inputs.hasTargets) {
            targets = result.getTargets();
            bestTarget = result.getBestTarget();
            inputs.bestTargetAmbiguity = bestTarget.getPoseAmbiguity();
            inputs.bestTargetArea = bestTarget.getArea();
            inputs.numTargets = targets.size();
//            targets.forEach(target -> ids.add(target.getFiducialId()));
//            inputs.targetsList = (Integer[]) ids.toArray();
        } else {
            targets = null;
            bestTarget = null;
            inputs.bestTargetAmbiguity = 1;
            inputs.bestTargetArea = 0;
            inputs.numTargets = 0;
        }
        if (inputs.hasEstimatedPose) {
            EstimatedRobotPose estimatedPose = poseUpdate.get();
            inputs.estimatedPose = estimatedPose.estimatedPose;
            inputs.timestampSeconds = estimatedPose.timestampSeconds;
        } else {
            inputs.estimatedPose = new Pose3d();
            inputs.timestampSeconds = 0;
        }
    }
}