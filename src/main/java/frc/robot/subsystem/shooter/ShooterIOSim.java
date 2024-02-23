package frc.robot.subsystem.shooter;

import frc.robot.subsystem.shooterV1.ShooterIOInputsV1AutoLogged;

public class ShooterIOSim extends ShooterIO {

    public ShooterIOSim(ShooterIOInputsAutoLogged input) {
        inputs = input;
    }

    @Override
    public void updateInputs() {

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
    public void setFlywheelBrake(boolean brake) {

    }
}
