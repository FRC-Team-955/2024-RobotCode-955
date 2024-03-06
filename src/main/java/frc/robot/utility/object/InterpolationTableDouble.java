package frc.robot.utility.object;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class InterpolationTableDouble {

    InterpolatingTreeMap<Double, Double> map = new InterpolatingTreeMap<>(
            (startValue, endValue, q) -> (q - startValue) / (endValue - startValue), (t, t1, v) -> t + ((t1 - t) * v));

    public InterpolationTableDouble(Map<Double, Double> table) {
        table.forEach(map::put);
    }

    public double get(double key) {
        return map.get(key);
    }
}
