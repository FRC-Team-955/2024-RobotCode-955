package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOReal extends ShooterIO{
    private final DigitalInput beamBreak;

    public ShooterIOReal(int pwmID) {
        beamBreak = new DigitalInput(pwmID);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.hasNote = !beamBreak.get();
    }
}
