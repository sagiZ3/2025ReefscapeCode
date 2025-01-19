package frc.lib.util.flippable;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.utilities.FieldConstants.FIELD_LENGTH;
import static frc.robot.utilities.FieldConstants.FIELD_WIDTH;

/**
 * A class that represents a {@link Pose2d} that can be flipped when the robot is on the red alliance.
 */
public class FlippablePose2d extends Flippable<Pose2d> {
    /**
     * Rotate the pose about the Y axis
     *
     * @param pose The pose to rotate
     * @return The rotated pose
     */
    public static Pose2d flipAboutYAxis(Pose2d pose) {
        final Pose2d flippedPose = FlippingUtil.flipFieldPose(pose);
        return new Pose2d(FIELD_LENGTH - flippedPose.getX(), flippedPose.getY(), Rotation2d.k180deg.minus(flippedPose.getRotation()));
    }

    /**
     * Rotate the pose about the X axis, including rotation.
     *
     * @param pose The pose to rotate
     * @return The rotated pose
     */
    public static Pose2d flipAboutXAxis(Pose2d pose) {
        final Pose2d flippedPose = FlippingUtil.flipFieldPose(pose);
        return new Pose2d(flippedPose.getX(), FIELD_WIDTH - flippedPose.getY(), flippedPose.getRotation().unaryMinus());
    }

    /**
     * Creates a new FlippablePose2d with the given x, y, and rotation.
     *
     * @param x                         the x value of the pose
     * @param y                         the y value of the pose
     * @param rotation                  the rotation of the pose
     * @param shouldFlipWhenRedAlliance should the pose be flipped when the robot is on the red alliance
     */
    public FlippablePose2d(double x, double y, Rotation2d rotation, boolean shouldFlipWhenRedAlliance) {
        this(new Pose2d(x, y, rotation), shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippablePose2d with the given translation and rotation.
     *
     * @param translation2d             the translation of the pose
     * @param rotationRadians           the rotation of the pose in radians
     * @param shouldFlipWhenRedAlliance should the pose be flipped when the robot is on the red alliance
     */
    public FlippablePose2d(Translation2d translation2d, double rotationRadians, boolean shouldFlipWhenRedAlliance) {
        this(new Pose2d(translation2d, new Rotation2d(rotationRadians)), shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippablePose2d with the given x, y, and rotation.
     *
     * @param nonFlippedPose            the pose when the robot is on the blue alliance
     * @param shouldFlipWhenRedAlliance should the pose be flipped when the robot is on the red alliance
     */
    public FlippablePose2d(Pose2d nonFlippedPose, boolean shouldFlipWhenRedAlliance) {
        super(nonFlippedPose, shouldFlipWhenRedAlliance);
    }

    /**
     * Gets the rotation value of the pose. The pose will be flipped if the robot is on the red alliance and {@link #shouldFlipWhenRedAlliance} is true.
     *
     * @return the rotation value of the pose
     */
    public FlippableRotation2d getRotation() {
        return new FlippableRotation2d(nonFlippedObject.getRotation(), shouldFlipWhenRedAlliance);
    }

    @Override
    protected Pose2d flip(Pose2d pose) {
        return FlippingUtil.flipFieldPose(pose);
    }
}