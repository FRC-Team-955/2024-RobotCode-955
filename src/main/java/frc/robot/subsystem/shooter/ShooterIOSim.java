package frc.robot.subsystem.shooter;

import frc.robot.subsystem.shooterV1.ShooterIOInputsV1AutoLogged;

public class ShooterIOSim extends ShooterIO {

    public ShooterIOSim(ShooterIOInputs input) {
        inputs = input;
    }

    @Override
    public void updateSensors() {

    }

    @Override
    public void updateApplications() {

    }

    @Override
    public void updatePivotController(double setpointDegrees, double feedforwardVolts) {

    }

    @Override
    public void updateFeedController(double setpointMetersPerSecond, double feedforwardVolts) {

    }

    @Override
    public void updateFlywheelController(double setpointMetersPerSecond, double feedforwardVoltsTop, double feedforwardVoltsBottom) {

    }

    @Override
    public void setPivotVolts(double volts) {

    }

    @Override
    public void setFeedVolts(double volts) {

    }

    @Override
    public void setFlywheelVolts(double volts) {

    }

    @Override
    public void zeroPivotRelative() {

    }

    @Override
    public void zeroPivotAbsolute() {

    }
}
