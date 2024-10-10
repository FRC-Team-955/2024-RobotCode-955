package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class FieldLocations {
    public static Translation2d getSpeaker() {return Util.flipIfNeeded(new Translation2d(0, 5.55));};
}
