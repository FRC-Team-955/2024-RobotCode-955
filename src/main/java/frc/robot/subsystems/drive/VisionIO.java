package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        public boolean hasTargets = false;
        public boolean hasEstimatedPose = false;
        public EstimatedRobotPose estimatedPose = null;
        public PhotonTrackedTarget bestTarget = null;
        public List<PhotonTrackedTarget> targets = null;
    }

    public void updateInputs(VisionIOInputs inputs) {
    }
}
