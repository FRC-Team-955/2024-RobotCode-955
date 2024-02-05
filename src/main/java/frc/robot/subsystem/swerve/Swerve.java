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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensor.pose.Gyro;
import frc.robot.sensor.pose.Odometry;
import frc.robot.utility.AngleUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;


/**
 * The Swerve Drivebase {@link Subsystem} for robot movement
 */
public class Swerve extends SubsystemBase {

    public static final Swerve instance = new Swerve();

    private enum State {
        Lock,
        Drive,
        Test
    }

    private State state = State.Lock;
    private double driveHeading = 0;
    private ChassisSpeeds controlSpeeds;
    private boolean pidHeadingControl = true;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.Swerve.modulePositions);

    private final PIDController headingPid = new PIDController(5, 0, 0);



    private Swerve() {
        AutoBuilder.configureHolonomic(
                Odometry::getPose,
                Odometry::resetPose,
                this::getSpeedsRelative,
                this::driveChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(0, 0, 0),
                        new PIDConstants(0, 0, 0),
                        Constants.Swerve.maxFreeSpeed,
                        Math.hypot(Constants.frameX - Constants.Swerve.wheelInset,
                                Constants.frameY - Constants.Swerve.wheelInset),
                        new ReplanningConfig()
                ),
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this
        );
        setBrakeMode(false);
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
                SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(controlSpeeds, 0.02));
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


    /**
     * Translates and rotates the swerve chassis based on percentage inputs
     * @param translation The desired translation of the robot in percentage of max speed
     * @param rotation The desired turn rate in percentage of max turn rate
     * @param fieldRelative Whether the translation should be relative to the field
     */
    public void drivePercents(Translation2d translation, double rotation, boolean fieldRelative) {
        driveSpeeds(translation.times(Constants.Swerve.maxFreeSpeed), rotation * Constants.Swerve.maxRotationSpeed, fieldRelative);
    }

    /**
     * Translates and rotates the swerve chassis based on velocity inputs
     * @param translation The desired translation of the robot in meters per second
     * @param rotation The desired turn rate in degrees per second
     * @param fieldRelative Whether the translation should be relative to the field
     */
    public void driveSpeeds(Translation2d translation, double rotation, boolean fieldRelative) {
        driveHeading += rotation * 0.02;
        driveChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
                pidHeadingControl ? headingPid.calculate(Gyro.getHeading().getDegrees(), driveHeading) : AngleUtil.degToRad(rotation),
                fieldRelative ? Gyro.getHeading() : Rotation2d.fromDegrees(0)));
    }

    /**
     * Translates and rotates the robot based on robot relative {@link ChassisSpeeds}
     * @param speeds The desired robot relative motion
     */
    public void driveChassisSpeeds(ChassisSpeeds speeds) {
        controlSpeeds = speeds;
        state = State.Drive;
    }

    /**
     * Prevents the swerve drivebase from moving on the field by facing all wheels inwards and
     * setting them to brake mode
     */
    public void lock() {
        state = State.Lock;
    }

    public void TEST() {
        state = State.Test;
    }

    /**
     * Sets whether the {@link Swerve#drivePercents(Translation2d, double, boolean)} and
     * {@link Swerve#driveSpeeds(Translation2d, double, boolean)} methods control the robot with
     * PID heading control or direct angular velocity control
     * @param usePidControl Whether to use PID heading control
     */
    public void setPidHeadingControl(boolean usePidControl) {
        pidHeadingControl = usePidControl;
    }

    /**
     * Sets whether to use brake mode on the swerve modules
     * @param brake Whether to use brake mode
     */
    public void setBrakeMode(boolean brake) {
        for (SwerveMod mod : SwerveMod.instance) {
            mod.setBrakeMode(brake);
        }
    }

    /**
     * Syncs the relative encoders on the angle motors with the absolute encoders on the swerve modules
     */
    public void syncEncoders() {
        for (SwerveMod mod : SwerveMod.instance) {
            mod.syncEncoders();
        }
    }



    /**
     * Assigns the given states to the swerve modules
     * @param states An array of {@link SwerveModuleState}s to be sent to their respective swerve modules
     *               as described by {@link SwerveMod#modId}
     */
    private void assignStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            SwerveMod.instance[i].setTargetState(states[i]);
        }
    }

    /**
     * Assigns the given states to the swerve modules based on the module's local heading, with 0 being the
     * @param states An array of {@link SwerveModuleState}s to be sent to their respective swerve modules
     *               as described by {@link SwerveMod#modId}
     */
    private void assignStatesLocalized(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            SwerveMod.instance[i].setTargetStateLocalized(states[i]);
        }
    }


    /**
     * Gets the current swerve module states
     * @return An array of {@link SwerveModuleState} representing the current state of each respective swerve module
     * as described by {@link SwerveMod#modId}
     */
    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
                SwerveMod.instance[0].getCurrentState(),
                SwerveMod.instance[1].getCurrentState(),
                SwerveMod.instance[2].getCurrentState(),
                SwerveMod.instance[3].getCurrentState()
        };
    }

    /**
     * Gets the current target swerve module states
     * @return An array of {@link SwerveModuleState} representing the current target state of each respective swerve module
     * as described by {@link SwerveMod#modId}
     */
    public SwerveModuleState[] getTargetStates() {
        return new SwerveModuleState[] {
                SwerveMod.instance[0].getTargetState(),
                SwerveMod.instance[1].getTargetState(),
                SwerveMod.instance[2].getTargetState(),
                SwerveMod.instance[3].getTargetState()
        };
    }

    /**
     * Gets the current swerve module states
     * @return An array of {@link SwerveModulePosition} representing the current state of each respective swerve module
     * as described by {@link SwerveMod#modId}
     */
    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                SwerveMod.instance[0].getCurrentPosition(),
                SwerveMod.instance[1].getCurrentPosition(),
                SwerveMod.instance[2].getCurrentPosition(),
                SwerveMod.instance[3].getCurrentPosition(),
        };
    }

    /**
     * Gets the robot relative speeds of the swerve drivebase
     * @return A {@link ChassisSpeeds} representing the current swerve drivebase speeds
     */
    public ChassisSpeeds getSpeedsRelative() {
        return kinematics.toChassisSpeeds(getStates());
    }
}
