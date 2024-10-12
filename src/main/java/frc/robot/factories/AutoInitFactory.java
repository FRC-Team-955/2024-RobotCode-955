package frc.robot.factories;

import choreo.auto.AutoLoop;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

import java.util.Optional;
import java.util.function.Supplier;

public class AutoInitFactory {
    public static Command get(AutoLoop loop, String initialTrajectoryName, Supplier<Optional<Pose2d>> initialPoseSupplier) {
        final var drive = Drive.get();

        return drive.setPose(() -> {
            var initialPose = initialPoseSupplier.get();
            if (initialPose.isPresent()) {
                return initialPose.get();
            } else {
                var msg = "No initial pose for trajectory " + initialTrajectoryName + "!";
                if (Constants.isSim)
                    throw new RuntimeException(msg);
                else
                    DriverStation.reportError(msg, false);

                loop.kill();
                return drive.getPose();
            }
        });
    }
}
