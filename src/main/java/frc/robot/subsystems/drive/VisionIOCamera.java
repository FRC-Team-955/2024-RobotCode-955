package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonPoseEstimator;

import java.util.List;

public class VisionIOCamera extends VisionIO {
    private final Transform3d robotToCam = new Transform3d(
            // Supposedly forward is positive x, left is positive y, up is positive z
            new Translation3d(Units.inchesToMeters(10.325), Units.inchesToMeters(-5.135), Units.inchesToMeters(12.458)),
            new Rotation3d(0,Units.degreesToRadians(30),0)
    );

    PhotonCamera cam;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cam,
            robotToCam
    );


    // ids are Front_Camera
    public VisionIOCamera(String id) {
        cam = new PhotonCamera(id);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        var result = cam.getLatestResult();
        boolean hasTargets = result.hasTargets();
        boolean hasEstimatedPose = photonPoseEstimator.update().isPresent();
        EstimatedRobotPose estimatedPose = null;
        PhotonTrackedTarget bestTarget = null;
        List<PhotonTrackedTarget> targets = null;
        if (hasTargets) {
            targets = result.getTargets();
            bestTarget = result.getBestTarget();
        }
        if(hasEstimatedPose) {
            estimatedPose = photonPoseEstimator.update().get();
        }
        inputs.hasEstimatedPose = hasEstimatedPose;
        inputs.hasTargets = hasTargets;
        inputs.estimatedPose = estimatedPose;
        inputs.bestTarget = bestTarget;
        inputs.targets = targets;
    }
}