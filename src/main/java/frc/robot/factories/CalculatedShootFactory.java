package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldLocations;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class CalculatedShootFactory {
    public static Command get(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        final var drive = Drive.get();
        final var shooter = Shooter.get();
        final var intake = Intake.get();

        return Commands.race(
                drive.driveJoystickPointShooterTowards(
                        xSupplier,
                        ySupplier,
                        FieldLocations.SPEAKER
                ),
                Commands.sequence(
                        Commands.waitUntil(() -> drive.pointingShooterTowardsPoint(FieldLocations.SPEAKER.get())),
                        shooter.shootDistance(drive::distanceToSpeaker)
                )
        );
    }
}
