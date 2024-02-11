package frc.robot.utility.information;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.sensor.pose.Odometry;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.Arrays;

public class StageDetector {

    private static final double[] redStageBounds = new double[] {
            0, 0,
            0, 0,
            0, 0
    };
    private static final double[] blueStageBounds = new double[] {
            0, 0,
            0, 0,
            0, 0
    };

    public static double distanceToIntersect() {
        double x = Odometry.getPose().getX();
        double y = Odometry.getPose().getY();

        Translation2d velocity = Odometry.getVelocity();

        double angle = velocity.getAngle().getRadians();
        double speed = velocity.getNorm();

        double dx = x + Math.cos(angle) * speed;
        double dy = y + Math.sin(angle) * speed;

        double[] intersects = new double[] {
                getIntersectDistance(redStageBounds[0], redStageBounds[1], redStageBounds[2], redStageBounds[3],
                        x, y, dx, dy),
                getIntersectDistance(redStageBounds[2], redStageBounds[3], redStageBounds[4], redStageBounds[5],
                        x, y, dx, dy),
                getIntersectDistance(redStageBounds[4], redStageBounds[5], redStageBounds[0], redStageBounds[1],
                        x, y, dx, dy),
                getIntersectDistance(blueStageBounds[0], blueStageBounds[1], blueStageBounds[2], blueStageBounds[3],
                        x, y, dx, dy),
                getIntersectDistance(blueStageBounds[2], blueStageBounds[3], blueStageBounds[4], blueStageBounds[5],
                        x, y, dx, dy),
                getIntersectDistance(blueStageBounds[4], blueStageBounds[5], blueStageBounds[0], blueStageBounds[1],
                        x, y, dx, dy),
        };

        return Arrays.stream(intersects).min().getAsDouble();
    }

    private static double getIntersectDistance(double ax, double ay, double bx, double by,
                                               double cx, double cy, double dx, double dy) {
        // https://sszczep.dev/blog/ray-casting-in-2d-game-engines
        double denom=(dx-cx)*(by-ay)-(bx-ax)*(dy-cy);
        double r=((bx-ax)*(cy-ay)-(cx-ax)*(by-ay))/denom;
        if (r+0.000001<0) return 100000;
        double s=((ax-cx)*(dy-cy)-(dx-cx)*(ay-cy))/denom;
        if (s+0.000001<0||s-0.000001>1) return 100000;
        return Math.hypot(s*(bx-ax)+ax,s*(by-ay)+ay);
    }
}
