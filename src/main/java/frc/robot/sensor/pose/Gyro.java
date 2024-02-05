package frc.robot.sensor.pose;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Robot;

/**
 * Manages the rotational position estimation of the robot
 */
public class Gyro {
    /**
     * The current sensor mode of the gyro
     * @see #Primary
     * @see #Secondary
     * @see #Estimation
     */
    public enum GyroMode {
        /**
         * The robot is using its primary gyro
         */
        Primary,
        /**
         * The robot has fallen back to its secondary gyro
         */
        Secondary,
        /**
         * The robot has fallen back on using pose estimation
         */
        Estimation
    }

    private static GyroMode mode = GyroMode.Primary;

    // Primary Gyro
    private static Pigeon2 pigeon;

    // Secondary Gyro
    private static AHRS navx;

    // Stores the estimated rotation in case both gyros fail
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

    /**
     * Updates the current rotational estimate of the robot position since the last tick
     * @param estimateDelta A {@link Rotation2d} representing the change in rotation
     *                      based on the {@link Odometry} since the last tick
     */
    public static void updateEstimateDelta(Rotation2d estimateDelta) {
        estimate = Rotation2d.fromDegrees(estimate.getDegrees() + estimateDelta.getDegrees());
    }

    /**
     * Returns the current mode of the gyro
     * @return The current {@link GyroMode} of the robot representing fallback to backup systems
     */
    public static GyroMode getMode() {
        return mode;
    }

    /**
     * Returns the current robot heading
     * @return A {@link Rotation2d} representing the current robot heading
     */
    public static Rotation2d getHeading() {
        return switch (mode) {
            case Primary -> Rotation2d.fromRotations(pigeon.getYaw().getValue());
            case Secondary -> new Rotation2d();
            case Estimation -> estimate;
        };
    }
}
