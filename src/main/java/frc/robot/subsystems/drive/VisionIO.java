package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean isConnected = false;
        public Pose3d pose3d;
    }

    public void updateInputs(VisionIOInputs inputs) {
    }
}
