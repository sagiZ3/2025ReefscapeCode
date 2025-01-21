package frc.robot.utilities;

import edu.wpi.first.math.geometry.*;

public class FieldConstants {
    private static final Transform2d ROBOT_TRANSFORM = new Transform2d(new Translation2d(0.5, 0), Rotation2d.fromDegrees(180));

    public static final Pose2d BLUE_BOTTOM_FEEDER_INTAKE_POSE = new Pose2d(new Translation2d(0.84319, 0.65078),
                    Rotation2d.fromDegrees(54)).transformBy(ROBOT_TRANSFORM);

    public static final double FIELD_WIDTH = 8.05;
    public static final double FIELD_LENGTH = 17.55;

    public static final double
            L1_HEIGHT_METERS = 0.457,
            L2_HEIGHT = 0.793,
            L3_HEIGHT = 1.196;
}