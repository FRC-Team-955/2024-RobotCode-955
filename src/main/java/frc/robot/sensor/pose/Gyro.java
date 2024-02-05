package frc.robot.sensor.pose;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Robot;

public class Gyro {
    public enum GyroMode {
        Primary,
        Secondary,
        Estimation
    }

    private static GyroMode mode = GyroMode.Primary;

    private static Pigeon2 pigeon;
    private static AHRS navx;

    private static Rotation2d estimate = Rotation2d.fromDegrees(0);

    static {
        if (Robot.isSimulation()) {
            mode = GyroMode.Estimation;
        } else {
            try {
                pigeon = new Pigeon2(0);
            }
            catch (Exception e) {
                mode = GyroMode.Secondary;
                System.out.println("Failed to connect to primary gyro");
                try {
                    navx = new AHRS(I2C.Port.kMXP);
                }
                catch (Exception ex) {
                    mode = GyroMode.Estimation;
                    System.out.println("Failed to connect to secondary gyro");
                }
            }
        }
    }

    public static void updateEstimateDelta(Rotation2d estimateDelta) {
        estimate = Rotation2d.fromDegrees(estimate.getDegrees() + estimateDelta.getDegrees());
    }

    public static GyroMode getMode() {
        return mode;
    }

    public static Rotation2d getHeading() {
        return switch (mode) {
            case Primary -> Rotation2d.fromRotations(pigeon.getYaw().getValue());
            case Secondary -> new Rotation2d();
            case Estimation -> estimate;
        };
    }
}
