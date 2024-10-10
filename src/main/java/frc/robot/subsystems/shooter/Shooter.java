package frc.robot.subsystems.shooter;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.drive.VisionIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
    protected static final ArmFeedforward PIVOT_FF = Constants.isReal ? new ArmFeedforward(0.574, 0.90, 0.65051, 0.21235) : new ArmFeedforward(0, 0.5, 0);
    protected static final PIDConstants PIVOT_PID = Constants.isReal ? new PIDConstants(0.15/*, 0.011*/) : new PIDConstants(2.5, 0);
    protected static final double PIVOT_GEAR_RATIO = 40;
    private static final Measure<Angle> PIVOT_ENCODER_OFFSET = Radians.of(0.0);
    private static final Measure<Angle> PIVOT_INITIAL_POSITION = Degrees.of(-90);
    private static final Measure<Angle> PIVOT_HOVER = Degrees.of(0);
    private static final Measure<Angle> PIVOT_WAIT_FOR_INTAKE = Degrees.of(-30);
    private static final Measure<Angle> PIVOT_HANDOFF = Degrees.of(-45);
    private static final Measure<Angle> PIVOT_SHOOT = Degrees.of(-50);
    private static final Measure<Angle> PIVOT_EJECT = Degrees.of(30);
    private static final Measure<Angle> PIVOT_AMP = Degrees.of(25);
    private static final Measure<Angle> PIVOT_SETPOINT_TOLERANCE = Degrees.of(1.5);

    protected static final SimpleMotorFeedforward FEED_FF = Constants.isReal ? new SimpleMotorFeedforward(0, 0) : new SimpleMotorFeedforward(0, 0.058);
    protected static final PIDConstants FEED_PID = Constants.isReal ? new PIDConstants(0.1, 0.0001) : new PIDConstants(0.1, 0);
    protected static final double FEED_GEAR_RATIO = 3;
    private static final Measure<Velocity<Angle>> FEED_SETPOINT_TOLERANCE = RPM.of(10);
    private static final double FEED_BEAM_BRAKE_DEBOUNCE = 0.05;

    private static final SimpleMotorFeedforward FLYWHEEL_TOP_FF = Constants.isReal ? new SimpleMotorFeedforward(0.23795, 0.011308, 0.0069895) : new SimpleMotorFeedforward(0, 0.058);
    private static final SimpleMotorFeedforward FLYWHEEL_BOTTOM_FF = Constants.isReal ? new SimpleMotorFeedforward(0.23279, 0.0113005, 0.0064997) : new SimpleMotorFeedforward(0, 0.058);
    private static final PIDConstants FLYWHEEL_TOP_PID = Constants.isReal ? new PIDConstants(0, 0) : new PIDConstants(0.1, 0);
    private static final PIDConstants FLYWHEEL_BOTTOM_PID = Constants.isReal ? new PIDConstants(0, 0) : new PIDConstants(0.1, 0);
    protected static final double FLYWHEEL_GEAR_RATIO = 1 / 2.0;
    private static final Measure<Velocity<Angle>> FLYWHEEL_SETPOINT_TOLERANCE = RPM.of(100);

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();


    public final LoggedDashboardNumber shootConfigurableSpeed = new LoggedDashboardNumber("Shoot Configurable Speed (RPM)", 0);
    public final LoggedDashboardNumber shootConfigurableAngle = new LoggedDashboardNumber("Shoot Configurable Angle (Degrees)", 0);

    private final Debouncer hasNoteDebouncer = new Debouncer(FEED_BEAM_BRAKE_DEBOUNCE);

    private final ArmFeedforward pivotFeedforward = PIVOT_FF;
    private Measure<Angle> pivotSetpoint = PIVOT_INITIAL_POSITION;
    public final SysIdRoutine pivotSysId;

    private final SimpleMotorFeedforward feedFeedforward = FEED_FF;
    private Measure<Velocity<Angle>> feedSetpoint = null;
    public final SysIdRoutine feedSysId;

    private final SimpleMotorFeedforward flywheelTopFeedforward = FLYWHEEL_TOP_FF;
    private final SimpleMotorFeedforward flywheelBottomFeedforward = FLYWHEEL_BOTTOM_FF;
    private Measure<Velocity<Angle>> flywheelsSetpoint = null;
    public final SysIdRoutine flywheelsSysId;

    private static Shooter instance;

    public static Shooter get() {
        return instance;
    }

    public Shooter(ShooterIO io) {
        if (instance != null)
            throw new RuntimeException("Duplicate subsystem created!");
        instance = this;

        this.io = io;

        io.pivotConfigurePID(PIVOT_PID);
        io.pivotSetPosition(PIVOT_INITIAL_POSITION.in(Radians));

        io.feedConfigurePID(FEED_PID);
        io.flywheelsConfigurePID(FLYWHEEL_TOP_PID, FLYWHEEL_BOTTOM_PID);

        pivotSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Shooter/Pivot/SysIdState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            pivotSetpoint = null;
                            io.pivotSetVoltage(voltage.in(Volts));
                        },
                        null,
                        this
                )
        );

        feedSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Shooter/Feed/SysIdState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            feedSetpoint = null;
                            io.feedSetVoltage(voltage.in(Volts));
                        },
                        null,
                        this
                )
        );

        flywheelsSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Shooter/Flywheels/SysIdState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            flywheelsSetpoint = null;
                            io.flywheelsSetVoltage(voltage.in(Volts));
                        },
                        null,
                        this
                )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Shooter", inputs);

        Logger.recordOutput("Shooter/Pivot/ClosedLoop", pivotSetpoint != null);
        if (pivotSetpoint != null) {
            Logger.recordOutput("Shooter/Pivot/Setpoint", pivotSetpoint);

            if (RobotState.isEnabled()) {
                var pivotSetpointRad = pivotSetpoint.in(Radians);
                var ffVolts = pivotFeedforward.calculate(pivotSetpointRad, 0);
                Logger.recordOutput("Shooter/Pivot/FFVolts", ffVolts);
                io.pivotSetSetpoint(pivotSetpointRad, ffVolts);
            }
        }

        Logger.recordOutput("Shooter/Feed/ClosedLoop", feedSetpoint != null);
        if (feedSetpoint != null) {
            Logger.recordOutput("Shooter/Feed/Setpoint", feedSetpoint);

            if (RobotState.isEnabled()) {
                var feedSetpointRadPerSec = feedSetpoint.in(RadiansPerSecond);
                var ffVolts = feedFeedforward.calculate(feedSetpointRadPerSec, 0);
                Logger.recordOutput("Shooter/Feed/FFVolts", ffVolts);
                io.feedSetSetpoint(feedSetpointRadPerSec, ffVolts);
            }
        }

        Logger.recordOutput("Shooter/Flywheels/ClosedLoop", flywheelsSetpoint != null);
        if (flywheelsSetpoint != null) {
            Logger.recordOutput("Shooter/Flywheels/Setpoint", flywheelsSetpoint);

            if (RobotState.isEnabled()) {
                var flywheelsSetpointRadPerSec = flywheelsSetpoint.in(RadiansPerSecond);
                var ffTopVolts = flywheelTopFeedforward.calculate(flywheelsSetpointRadPerSec, 0);
                var ffBottomVolts = flywheelBottomFeedforward.calculate(flywheelsSetpointRadPerSec, 0);
                Logger.recordOutput("Shooter/Flywheels/FFTopVolts", ffTopVolts);
                Logger.recordOutput("Shooter/Flywheels/FFBottomVolts", ffBottomVolts);
                io.flywheelsSetSetpoint(flywheelsSetpointRadPerSec, ffTopVolts, ffBottomVolts);
            }
        }
    }

    @AutoLogOutput
    public boolean hasNoteDebounced() {
        return hasNoteDebouncer.calculate(inputs.hasNote);
    }

    public boolean alreadyAtHover() {
        return inputs.pivotPositionRad <= PIVOT_HOVER.plus(Degrees.of(10)).in(Radians);
    }

    private boolean pivotAtSetpoint() {
        return Math.abs(inputs.pivotPositionRad - pivotSetpoint.in(Radians)) <= PIVOT_SETPOINT_TOLERANCE.in(Radians);
    }

    private boolean flywheelsAtSetpoint() {
        return Math.abs(inputs.flywheelTopVelocityRadPerSec - flywheelsSetpoint.in(RadiansPerSecond)) <= FLYWHEEL_SETPOINT_TOLERANCE.in(RadiansPerSecond) &&
                Math.abs(inputs.flywheelBottomVelocityRadPerSec - flywheelsSetpoint.in(RadiansPerSecond)) <= FLYWHEEL_SETPOINT_TOLERANCE.in(RadiansPerSecond);
    }

    private Command pivotSetpoint(Measure<Angle> setpoint) {
        return startEnd(
                () -> pivotSetpoint = setpoint,
                () -> {
                }
        ).until(this::pivotAtSetpoint);
    }

    private Command feedPercent(double percent) {
        return startEnd(
                () -> io.feedSetVoltage(percent * 12.0),
                () -> io.feedSetVoltage(0)
        );
    }

    private void flywheelsPercent(double percent) {
        io.flywheelsSetVoltage(percent * 12.0);
    }

    private void flywheelsStop() {
        io.flywheelsSetVoltage(0);
    }

    public Command pivotHover() {
        return pivotSetpoint(PIVOT_HOVER);
    }

    public Command pivotWaitForIntake() {
        return pivotSetpoint(PIVOT_WAIT_FOR_INTAKE);
    }

    public Command pivotHandoff() {
        return pivotSetpoint(PIVOT_HANDOFF);
    }

    public Command pivotShoot() {
        return pivotSetpoint(PIVOT_SHOOT);
        /*
        return startEnd(
                () -> pivot.pivotSetSetpoint(Degrees.of(shoot_angle.get())),
                () -> {
                }
        ).until(pivot::atSetpoint);
        */
    }

    private Command pivotEject() {
        return pivotSetpoint(PIVOT_EJECT);
    }

    private Command pivotAmp() {
        return pivotSetpoint(PIVOT_AMP);
    }

    public Command feedHandoff() {
        return feedPercent(0.3).until(this::hasNoteDebounced);
    }

    public Command shootPercentUntimed(double percent) {
        return new RunCommand(() -> flywheelsPercent(percent));
    }

    private Command shootPercent(double percent, double spinupTime) {
        return Commands.sequence(
                startEnd(() -> flywheelsPercent(percent), () -> {
                }).withTimeout(spinupTime),
                feedPercent(1).withTimeout(0.6)
        ).finallyDo(this::flywheelsStop);
    }

    public Command amp() {
        return Commands.sequence(
                pivotAmp(),
                Commands.runOnce(() -> shootPercent(0.25, 0.25).schedule())
        );
    }

    public Command shoot() {
        return Commands.sequence(
                pivotShoot(),
                Commands.runOnce(() -> shootPercent(0.5, 0.75).schedule())
        );
    }

    public Command eject() {
        return pivotEject().andThen(startEnd(() -> {
            flywheelsPercent(0.5);
            io.feedSetVoltage(12);
        }, () -> {
            flywheelsStop();
            io.feedSetVoltage(0);
        }));
    }

    public Command shootConfigurable() {
        return Commands.sequence(
                startEnd(
                        () -> pivotSetpoint = Degrees.of(shootConfigurableAngle.get()),
                        () -> {
                        }
                ).until(this::pivotAtSetpoint),
                startEnd(
                        () -> flywheelsSetpoint = RPM.of(shootConfigurableSpeed.get()),
                        () -> {
                        }
                ).until(this::flywheelsAtSetpoint),
                feedPercent(1).withTimeout(0.6)
        ).finallyDo(() -> {
            pivotSetpoint = Degrees.of(0);
            flywheelsSetpoint = RPM.of(0);
        });
    }

    public Command shootDistance(double distance) {
        return Commands.sequence(
                startEnd(
                        () -> pivotSetpoint = Degrees.of(ShooterRegression.getAngle(distance)),
                        () -> {}
                ).until(this::pivotAtSetpoint),
                startEnd(
                        () -> flywheelsSetpoint = RPM.of(ShooterRegression.getSpeed(distance)),
                        () -> {}
                ).until(this::flywheelsAtSetpoint),
                feedPercent(1).withTimeout(0.6)
        ).finallyDo(() -> {
            pivotSetpoint = Degrees.of(0);
            flywheelsSetpoint = RPM.of(0);
        });
    }
}
