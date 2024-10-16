// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.dashboard.DashboardSubsystem;
import frc.robot.dashboard.TuningDashboardNumber;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class WheelRadiusCharacterization extends Command {
    private static final TuningDashboardNumber characterizationSpeed = new TuningDashboardNumber(DashboardSubsystem.DRIVE, "Wheel Radius Characterization Rotation Speed (rad per sec)", 0.1);
    private static final DoubleSupplier gyroYawRadsSupplier = () -> RobotState.get().getRotation().getRadians();

    @RequiredArgsConstructor
    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int value;
    }

    private final Drive drive = Drive.get();
    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

    private double currentEffectiveWheelRadius = 0.0;

    public WheelRadiusCharacterization(Direction omegaDirection) {
        this.omegaDirection = omegaDirection;
    }

    private double[] getPositions() {
        return Arrays.stream(drive.getModulePositions())
                .mapToDouble((m) -> m.angle.getRadians())
                .toArray();
    }

    @Override
    public void initialize() {
        // Reset
        drive.startCharacterization();
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        startWheelPositions = getPositions();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        // Run drive at velocity
        var omega = omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get());
        drive.runVelocity(new ChassisSpeeds(0, 0, omega));

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        double averageWheelPosition = 0.0;
        double[] wheelPositiions = getPositions();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * Drive.DRIVE_BASE_RADIUS) / averageWheelPosition;
        Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
        Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
        Logger.recordOutput(
                "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
                Units.metersToInches(currentEffectiveWheelRadius));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopCharacterization();
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        } else {
            System.out.println(
                    "Effective Wheel Radius: "
                            + Units.metersToInches(currentEffectiveWheelRadius)
                            + " inches");
        }
    }
}
