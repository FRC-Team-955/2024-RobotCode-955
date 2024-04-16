package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.AutoLogOutputManager;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

public class Util {
    public static void registerFieldsForAutoLogOutput(Object... roots) {
        for (var root : roots) {
            try {
                Method method =
                        AutoLogOutputManager.class.getDeclaredMethod("registerFields", Object.class);
                method.setAccessible(true);
                method.invoke(null, root);
            } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
}
