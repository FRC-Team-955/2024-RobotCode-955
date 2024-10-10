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

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionIOCamera extends VisionIO {
    private final Transform3d robotToCam = new Transform3d(
            // Supposedly forward is positive x, left is positive y, up is positive z
            new Translation3d(Units.inchesToMeters(-10.325), Units.inchesToMeters(-5.135), Units.inchesToMeters(12.458)),
            new Rotation3d(0, -Units.degreesToRadians(30), Math.PI)
    );

    PhotonCamera cam;

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    PhotonPoseEstimator photonPoseEstimator;

    EstimatedRobotPose estimatedPose = null;
    List<PhotonTrackedTarget> targets = null;
    PhotonTrackedTarget bestTarget = null;
    ArrayList<Integer> ids;


    // ids are Shooter_Cam
    public VisionIOCamera(String id) {
        cam = new PhotonCamera(id);
        photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cam,
                robotToCam
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        var result = cam.getLatestResult();
        boolean hasTargets = result.hasTargets();
        Optional<EstimatedRobotPose> poseUpdate = photonPoseEstimator.update();
        boolean hasEstimatedPose = poseUpdate.isPresent();
        if (hasTargets) {
            targets = result.getTargets();
            bestTarget = result.getBestTarget();
            inputs.bestTargetAmbiguity = bestTarget.getPoseAmbiguity();
//            targets.forEach(target -> ids.add(target.getFiducialId()));
//            inputs.targetsList = (Integer[]) ids.toArray();
        } else {
            targets = null;
            bestTarget = null;
            inputs.bestTargetAmbiguity = 1;
        }
        if (hasEstimatedPose) {
            estimatedPose = poseUpdate.get();
            inputs.estimatedPose = estimatedPose.estimatedPose;
            inputs.timestampSeconds = estimatedPose.timestampSeconds;
        } else {
            inputs.estimatedPose = new Pose3d();
            inputs.timestampSeconds = 0;
        }
        inputs.hasEstimatedPose = hasEstimatedPose;
        inputs.hasTargets = hasTargets;
    }
}