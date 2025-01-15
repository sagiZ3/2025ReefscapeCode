package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldLocations {
    public static final Pose3d BLUE_TOP_FEEDER = new Pose3d(new Translation3d(0.84319, 7.41395, 0.9525),
            new Rotation3d(0, 0, Math.toRadians(-54)));
    public static final Pose3d BLUE_BOTTOM_FEEDER = new Pose3d(new Translation3d(0.84319, 0.65078, 0.9525),
            new Rotation3d(0, 0, Math.toRadians(54)));

    public static final double FIELD_WIDTH = 8.05;
    public static final double FIELD_LENGTH = 17.55;
}