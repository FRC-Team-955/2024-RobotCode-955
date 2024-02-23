package frc.robot.subsystem.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.subsystem.shooterV1.ShooterV1;
import frc.robot.utility.conversion.AngleUtil;
import frc.robot.utility.conversion.ObjectUtil;
import frc.robot.utility.information.StageDetector;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;


/**
 * The Swerve Drivebase {@link Subsystem} for robot movement
 */
public class Swerve extends SubsystemBase {

    public static Swerve instance;

    private enum State {
        Lock,
        Drive,
        Test
    }

    private State state = State.Drive;
    private double driveHeading = 0;
    private ChassisSpeeds controlSpeeds = new ChassisSpeeds();
    private boolean pidHeadingControl = true;
    private double targetHeading = -1;
    private Supplier<Optional<Rotation2d>> headingController = Optional::empty;
    private boolean stageAware = false;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.Swerve.modulePositions);

    private final PIDController headingPid = new PIDController(5, 0, 0);



    public Swerve() {
        instance = this;

        SwerveMod.init();

        AutoBuilder.configureHolonomic(
                Odometry::getPose,
                Odometry::resetPose,
                this::getSpeedsRelativeI,
                this::driveChassisSpeedsI,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(0, 0, 0),
                        new PIDConstants(0, 0, 0),
                        Constants.Swerve.Constraints.maxFreeSpeed,
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
        PPHolonomicDriveController.setRotationTargetOverride(() -> {
            return headingController.get();
        });
        setBrakeModeI(true);
    }

//    /**
//     * Initializes the subsystem
//     */
//    public static void init() {
//        if (instance == null) {
//            new Swerve();
//        }
//    }



    public void updateInputs() {
        SwerveMod.instance[0].updateInputs();
        SwerveMod.instance[1].updateInputs();
        SwerveMod.instance[2].updateInputs();
        SwerveMod.instance[3].updateInputs();

        SwerveModulePosition[] deltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            deltas[i] = SwerveMod.instance[i].getPositionDelta();
        }

        if (headingController.get().isPresent())
            driveHeading = Gyro.getHeading().getDegrees();

        Gyro.updateEstimateDelta(Rotation2d.fromRadians(kinematics.toTwist2d(deltas).dtheta));
        Odometry.updateEstimatePositions();
//        VisionAprilTag.update();
    }

    @Override
    public void periodic() {

        if (targetHeading >= 0 && Math.abs(targetHeading - AngleUtil.unsignedRangeDegrees(
                Gyro.getHeading().getDegrees())) <= Constants.Swerve.Tolerances.heading) {
            targetHeading = -1;
            headingController = Optional::empty;
        }

        switch (state) {
            case Lock -> {
                for (SwerveMod mod : SwerveMod.instance) {
                    mod.setTargetStateLocalized(new SwerveModuleState(
                            0, Rotation2d.fromDegrees(315)));
                    mod.setBrakeMode(true);
                }
            }
            case Drive -> {
                SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                        ChassisSpeeds.discretize(stageAware && !ShooterV1.isStageSafe() ?
                                ObjectUtil.limitChassisSpeeds(controlSpeeds, Constants.Swerve.Constraints.maxFreeSpeed,
                                        1 - getStageLockI()) : controlSpeeds, 0.02));
                assignStates(states);
            }
            case Test -> {}
        }

        Logger.recordOutput("SwerveStates/Target", getTargetStatesI());
        Logger.recordOutput("SwerveStates/Current", getStatesI());
    }


    /**
     * Translates and rotates the swerve chassis based on percentage inputs
     * @param translation The desired translation of the robot in percentage of max speed
     * @param rotation The desired turn rate in percentage of max turn rate
     * @param fieldRelative Whether the translation should be relative to the field
     */
    public static void drivePercents(Translation2d translation, double rotation, boolean fieldRelative) {
        instance.drivePercentsI(translation, rotation, fieldRelative);
    }
    private void drivePercentsI(Translation2d translation, double rotation, boolean fieldRelative) {
        driveSpeedsI(translation.times(Constants.Swerve.Constraints.maxFreeSpeed), rotation *
                Constants.Swerve.Constraints.maxRotationSpeed, fieldRelative);
    }

    /**
     * Translates and rotates the swerve chassis based on velocity inputs
     * @param translation The desired translation of the robot in meters per second
     * @param rotation The desired turn rate in degrees per second
     * @param fieldRelative Whether the translation should be relative to the field
     */
    public static void driveSpeeds(Translation2d translation, double rotation, boolean fieldRelative) {
        instance.driveSpeedsI(translation, rotation, fieldRelative);
    }
    private void driveSpeedsI(Translation2d translation, double rotation, boolean fieldRelative) {
        if (rotation != 0)
            cancelTargetHeading();
        driveHeading += rotation * 0.02;
        driveChassisSpeedsI(ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
                pidHeadingControl || headingController.get().isPresent() ? headingPid.calculate(
                        Gyro.getHeading().getDegrees(), headingController.get().isEmpty() ? driveHeading :
                                headingController.get().get().getDegrees()) : AngleUtil.degToRad(rotation),
                fieldRelative ? Gyro.getHeading() : Rotation2d.fromDegrees(0)));
    }

    /**
     * Translates and rotates the robot based on robot relative {@link ChassisSpeeds}
     * @param speeds The desired robot relative motion
     */
    public static void driveChassisSpeeds(ChassisSpeeds speeds) {
        instance.driveChassisSpeedsI(speeds);
    }
    private void driveChassisSpeedsI(ChassisSpeeds speeds) {
        controlSpeeds = speeds;
        state = State.Drive;
    }

    /**
     * Prevents the swerve drivebase from moving on the field by facing all wheels inwards and
     * setting them to brake mode
     */
    public static void lock() {
        instance.lockI();
    }
    private void lockI() {
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
    public static void setPidHeadingControl(boolean usePidControl) {
        instance.setPidHeadingControlI(usePidControl);
    }
    private void setPidHeadingControlI(boolean usePidControl) {
        pidHeadingControl = usePidControl;
    }

    /**
     * Sets the target heading of the robot, which will be overridden by a drive input greater than 0
     * @param heading The target heading in degrees
     */
    public static void setTargetHeading(double heading) {
        instance.setTargetHeadingI(heading);
    }
    private void setTargetHeadingI(double heading) {
        targetHeading = heading;
        headingController = () -> {
            return Optional.of(Rotation2d.fromDegrees(AngleUtil.unsignedRangeDegrees(heading)));
        };
    }
    private void cancelTargetHeading() {
        if (targetHeading == 1)
            return;
        targetHeading = -1;
        driveHeading = Gyro.getHeading().getDegrees();
        headingController = Optional::empty;
    }
    /**
     * Sets the heading override
     * @param rotationController A {@link Supplier<Rotation2d>} for the override heading
     */
    public static void setHeadingOverride(Supplier<Rotation2d> rotationController) {
        instance.setHeadingControllerI(() -> {
            return Optional.of(rotationController.get());
        });
    }
    /**
     * Disables the current heading override
     */
    public static void disableHeadingOverride() {
        instance.setHeadingControllerI(Optional::empty);
    }
    /**
     * Sets the heading controller. An empty optional will result in normal control,
     * and a provided {@link Rotation2d} will result in a heading override
     * @param rotationController A {@link Supplier<Optional<Rotation2d>>} for the desired heading
     */
    public static void setHeadingController(Supplier<Optional<Rotation2d>> rotationController) {
        instance.setHeadingControllerI(rotationController);
    }
    private void setHeadingControllerI(Supplier<Optional<Rotation2d>> rotationController) {
        cancelTargetHeading();
        headingController = rotationController;
    }

    /**
     * Sets whether to use brake mode on the swerve modules
     * @param brake Whether to use brake mode
     */
    public static void setBrakeMode(boolean brake) {
        instance.setBrakeModeI(brake);
    }
    private void setBrakeModeI(boolean brake) {
        for (SwerveMod mod : SwerveMod.instance) {
            mod.setBrakeMode(brake);
        }
    }

    /**
     * Syncs the relative encoders on the angle motors with the absolute encoders on the swerve modules
     */
    public static void syncEncoders() {
        instance.syncEncodersI();
    }
    private void syncEncodersI() {
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
     * Gets the percent by which the target velocity should be reduced in order to prevent hitting the stage
     * @return The percent of velocity reduction
     */
    public static double getStageLock() {
        return instance.getStageLockI();
    }
    private double getStageLockI() {
        double dist = StageDetector.distanceToIntersect();
        ChassisSpeeds speeds = controlSpeeds;
        double distLock = MathUtil.inverseInterpolate(Constants.Stage.Safety.warningDistance,
                Constants.Stage.Safety.dangerDistance, dist);
        double timeLock = MathUtil.inverseInterpolate(Constants.Stage.Safety.warningTime,
                Constants.Stage.Safety.dangerTime,
                dist / Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        return Math.min(distLock, timeLock);
    }

    /**
     * Gets the current swerve module states
     * @return An array of {@link SwerveModuleState} representing the current state of each respective swerve module
     * as described by {@link SwerveMod#modId}
     */
    public static SwerveModuleState[] getStates() {
        return instance.getStatesI();
    }
    private SwerveModuleState[] getStatesI() {
        return new SwerveModuleState[] {
                SwerveMod.instance[0].getCurrentState(),
                SwerveMod.instance[1].getCurrentState(),
                SwerveMod.instance[2].getCurrentState(),
                SwerveMod.instance[3].getCurrentState()
        };
    }

    /**
     * Gets the current target swerve module states
     * @return An array of {@link SwerveModuleState} representing the current target state of each
     * respective swerve module as described by {@link SwerveMod#modId}
     */
    public static SwerveModuleState[] getTargetStates() {
        return instance.getTargetStatesI();
    }
    private SwerveModuleState[] getTargetStatesI() {
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
    public static SwerveModulePosition[] getPositions() {
        return instance.getPositionsI();
    }
    private SwerveModulePosition[] getPositionsI() {
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
    public static ChassisSpeeds getSpeedsRelative() {
        return instance.getSpeedsRelativeI();
    }
    private ChassisSpeeds getSpeedsRelativeI() {
        return kinematics.toChassisSpeeds(getStatesI());
    }
}
