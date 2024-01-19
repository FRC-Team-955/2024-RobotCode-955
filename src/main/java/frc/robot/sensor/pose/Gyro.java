package frc.robot.sensor.pose;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

public class Gyro {
    public enum Mode {
        Primary,
        Secondary,
        Estimation
    }

    public static Mode mode = Mode.Primary;

    private static Pigeon2 pigeon;
    // TODO - Implement NavX support once the library updates to 2024

    static {
        if (Robot.isSimulation()) {
            mode = Mode.Estimation;
        } else {
            try {
                pigeon = new Pigeon2(0);
            }
            catch (Exception e) {
                mode = Mode.Secondary;
                System.out.println("Failed to connect to primary gyro");
//                try {
//                    // Instantiate NavX
//                }
//                catch (Exception ex) {
                    mode = Mode.Estimation;
                    System.out.println("Failed to connect to secondary gyro");
//                }
            }
        }
    }

    public static Rotation2d getHeading() {
        return new Rotation2d();
    }
}
