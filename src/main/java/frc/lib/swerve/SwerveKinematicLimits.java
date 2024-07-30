package frc.lib.swerve;

public record SwerveKinematicLimits(
        double maxDriveVelocityMetersPerSec,
        double maxDriveAccelerationMetersPerSecPerSec,
        double maxSteeringVelocityRadPerSec
) {
}
