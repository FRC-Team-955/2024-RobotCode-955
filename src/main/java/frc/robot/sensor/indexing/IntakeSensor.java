package frc.robot.sensor.indexing;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.simulation.UltrasonicSim;

public class IntakeSensor {

    private static final Ultrasonic sensor;

    static {
        sensor = new Ultrasonic(0, 0);
    }
}
