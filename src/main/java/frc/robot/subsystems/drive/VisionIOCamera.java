package frc.robot.subsystems.drive;

import org.photonvision.PhotonCamera;

public class VisionIOCamera extends VisionIO {
    PhotonCamera cam;

    public VisionIOCamera(String id) {
        cam = new PhotonCamera(id);
    }
}