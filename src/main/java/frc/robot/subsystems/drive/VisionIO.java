package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public class VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean isConnected = false;
        public Pose3d pose3d;
    }

    public void updateInputs(VisionIOInputs inputs) {
    }
}
