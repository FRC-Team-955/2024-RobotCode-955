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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensor.pose.Gyro;
import frc.robot.sensor.pose.Odometry;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Swerve extends SubsystemBase {

    public static final Swerve instance = new Swerve();

    private enum State {
        Lock,
        Drive,
        DriveRobotRelative,
        Path,
        Test
    }

    private State state = State.Lock;

    private Translation2d driveTranslation = new Translation2d();
    private double driveHeadingTranslation = 0;
    private double driveHeading = 0;
    private ChassisSpeeds pathSpeeds;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.Swerve.modulePositions);

    private final PIDController headingPid = new PIDController(5, 0, 0);



    private Swerve() {
        AutoBuilder.configureHolonomic(
                Odometry::getPose,
                Odometry::resetPose,
                this::getSpeedsRelative,
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

    @Override
    public void periodic() {
        switch (state) {
            case Lock -> {
                for (SwerveMod mod : SwerveMod.instance) {
                    mod.setTargetStateLocalized(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
                    mod.setBrakeMode(true);
                }
            }
            case Drive -> {
                for (SwerveMod mod : SwerveMod.instance) {
                    mod.setBrakeMode(true);
                }
                driveHeading += driveHeadingTranslation * Constants.Input.headingRateOfChange * 0.02;
                Rotation2d heading = Gyro.getHeading();
                SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(driveTranslation.getX(), driveTranslation.getY(),
                        driveHeadingTranslation * 6, heading));
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
            case Test -> {
                assignStates(new SwerveModuleState[]{
                        new SwerveModuleState(15, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(15, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(15, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(15, Rotation2d.fromDegrees(0))
                });
            }
        }

        SwerveModulePosition[] deltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            deltas[i] = SwerveMod.instance[i].getPositionDelta();
        }

        Gyro.updateEstimateDelta(Rotation2d.fromRadians(kinematics.toTwist2d(deltas).dtheta));
        Odometry.updateEstimatePositions();

        Logger.recordOutput("SwerveStates/Target", getTargetStates());
        Logger.recordOutput("SwerveStates/Current", getStates());
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

    public void lock() {
        state = State.Lock;
    }

    public void TEST() {
        state = State.Test;
    }



    private void assignStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            SwerveMod.instance[i].setTargetState(states[i]);
        }
    }

    public void assignStatesLocalized(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            SwerveMod.instance[i].setTargetStateLocalized(states[i]);
        }
    }

    public void setBrakeMode(boolean brake) {
        for (SwerveMod mod : SwerveMod.instance) {
            mod.setBrakeMode(brake);
        }
    }



    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
                SwerveMod.instance[0].getCurrentState(),
                SwerveMod.instance[1].getCurrentState(),
                SwerveMod.instance[2].getCurrentState(),
                SwerveMod.instance[3].getCurrentState()
        };
    }

    public SwerveModuleState[] getTargetStates() {
        return new SwerveModuleState[] {
                SwerveMod.instance[0].getTargetState(),
                SwerveMod.instance[1].getTargetState(),
                SwerveMod.instance[2].getTargetState(),
                SwerveMod.instance[3].getTargetState()
        };
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                SwerveMod.instance[0].getCurrentPosition(),
                SwerveMod.instance[1].getCurrentPosition(),
                SwerveMod.instance[2].getCurrentPosition(),
                SwerveMod.instance[3].getCurrentPosition(),
        };
    }

    public ChassisSpeeds getSpeedsRelative() {
        return kinematics.toChassisSpeeds(getStates());
    }
}
