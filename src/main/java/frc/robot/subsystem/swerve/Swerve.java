package frc.robot.subsystem.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensor.pose.Gyro;
import frc.robot.sensor.pose.Odometry;
import frc.robot.utility.AngleUtil;

import java.util.Optional;

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
    private ChassisSpeeds pathSpeeds;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.Swerve.modulePositions);

    private final PIDController headingPid = new PIDController(.1, 0, 0);

    private Swerve() {
        AutoBuilder.configureHolonomic(
                Odometry::getPose,
                Odometry::updatePostEstimateTrustworthy,
                Odometry::getSpeedsRelative,
                this::followPathChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(0, 0, 0),
                        new PIDConstants(0, 0, 0),
                        Constants.Swerve.maxFreeSpeed,
                        Math.sqrt((Constants.frameX - Constants.Swerve.wheelInset) * (Constants.frameX - Constants.Swerve.wheelInset) +
                            (Constants.frameY - Constants.Swerve.wheelInset) * (Constants.frameY - Constants.Swerve.wheelInset)),
                        new ReplanningConfig()
                ),
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this
        );
    }

    public void drive(Translation2d translation, double rotation) {
        driveTranslation = translation;
        driveHeadingTranslation = rotation;
        state = State.Drive;
    }

    public void followPathChassisSpeeds(ChassisSpeeds speeds) {
        state = State.Path;
        pathSpeeds = speeds;
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
            case Path -> {
                SwerveModuleState[] states = kinematics.toSwerveModuleStates(pathSpeeds);
                assignStates(states);
            }
        }

        Odometry.updateEstimateChassisSpeeds(getSpeeds(), getSpeedsRelative());
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

    private SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
                SwerveMod.instance[0].getCurrentState(),
                SwerveMod.instance[1].getCurrentState(),
                SwerveMod.instance[2].getCurrentState(),
                SwerveMod.instance[3].getCurrentState()
        };
    }

    private ChassisSpeeds getSpeeds() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getStates());
        return new ChassisSpeeds(
                speeds.vxMetersPerSecond * Math.cos(Gyro.getHeading().getRadians()) +
                speeds.vyMetersPerSecond * Math.sin(Gyro.getHeading().getRadians()),
                speeds.vxMetersPerSecond * Math.sin(Gyro.getHeading().getRadians()) +
                speeds.vyMetersPerSecond * Math.cos(Gyro.getHeading().getRadians()),
                speeds.omegaRadiansPerSecond
        );
    }

    private ChassisSpeeds getSpeedsRelative() {
        return kinematics.toChassisSpeeds(getStates());
    }
}
