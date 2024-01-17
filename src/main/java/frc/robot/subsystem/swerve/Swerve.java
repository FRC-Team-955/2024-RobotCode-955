package frc.robot.subsystem.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensor.gyro.Gyro;
import frc.robot.utility.AngleUtil;

public class Swerve extends SubsystemBase {

    public static final Swerve instance = new Swerve();

    private enum State {
        Lock,
        Drive,
        DriveRobotRelative,
        Path
    }

    private State state = State.Lock;

    private Translation2d driveTranslation = new Translation2d();
    private double driveHeadingTranslation = 0;
    private double driveHeading = 0;
    private Translation2d centerOfRot = new Translation2d();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.Swerve.modulePositions);

    private final PIDController headingPid = new PIDController(.1, 0, 0);

    private Swerve() {

    }

    @Override
    public void periodic() {
        for (SwerveMod mod : SwerveMod.instance) {
            mod.updateInputs();
        }

        switch (state) {
            case Lock -> {
                for (SwerveMod mod : SwerveMod.instance) {
                    mod.setTargetStateLocalized(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
                    mod.setIdleMode(SwerveMod.IdleMode.Brake);
                }
            }
            case Drive -> {
                driveHeading += driveHeadingTranslation * Constants.Input.headingRateOfChange * 0.02;
                Rotation2d heading = Gyro.getHeading();
                SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(driveTranslation.getX(), driveTranslation.getY(),
                        headingPid.calculate(heading.getRadians(), AngleUtil.degToRad(driveHeading)), heading));
                assignStates(states);
            }
            case DriveRobotRelative -> {
                driveHeading += driveHeadingTranslation * Constants.Input.headingRateOfChange * 0.02;
                Rotation2d heading = Gyro.getHeading();
                SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(driveTranslation.getX(), driveTranslation.getY(),
                        headingPid.calculate(heading.getDegrees(), driveHeading), Rotation2d.fromDegrees(0)));
                assignStates(states);
            }
            // TODO - Implement Path Planner path following
            case Path -> {

            }
        }
    }

    private void assignStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            SwerveMod.instance[i].setTargetState(states[i]);
        }
    }

    private void assignStatesLocalized(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            SwerveMod.instance[i].setTargetStateLocalized(states[i]);
        }
    }
}
