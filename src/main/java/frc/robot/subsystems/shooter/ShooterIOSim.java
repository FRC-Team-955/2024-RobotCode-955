package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;

public class ShooterIOSim extends ShooterIO {
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.hasNote = Timer.getFPGATimestamp() % 10 > 5;
    }
}
