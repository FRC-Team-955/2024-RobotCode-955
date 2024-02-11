package frc.robot.utility.information;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Input {
    private static final CommandXboxController driver;

    static {
        driver = new CommandXboxController(0);
    }

    public static double driveY() {
        return driver.getLeftY();
    }
    public static double driveX() {
        return driver.getLeftX();
    }
    public static double rotation() { return driver.getRightX(); }

    public static Trigger button1() {
        return driver.x();
    }
    public static Trigger button2() {
        return driver.y();
    }
    public static Trigger button3() {
        return driver.b();
    }

}
