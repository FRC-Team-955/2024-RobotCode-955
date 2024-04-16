package frc.lib.subsystems.wheel;

/**
 * Generic flywheel or roller
 */
public class Wheel {
    private final String inputsName;
    public final WheelIOInputsAutoLogged inputs = new WheelIOInputsAutoLogged();
    public final WheelIO io;

    public Wheel(String inputName, WheelIO io) {
        this.inputsName = inputName;
        this.io = io;
    }
}
