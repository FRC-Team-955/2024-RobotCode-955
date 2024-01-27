package frc.robot.subsystem.climber;

public class ClimberIOSim extends ClimberIO {

    public ClimberIOSim(ClimberIOValuesAutoLogged input) {
        inputs = input;
    }

    private ClimberIOValuesAutoLogged inputs;

    @Override
    public void updateInputs(ClimberIOValues values) {

    }

    @Override
    public void moveLeft(double percent) {

    }

    @Override
    public void moveRight(double percent) {

    }

    @Override
    public void setLeftIdle(Climber.IdleMode mode) {

    }

    @Override
    public void setRightIdle(Climber.IdleMode mode) {

    }
}
