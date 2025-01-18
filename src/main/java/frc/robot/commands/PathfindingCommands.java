package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveCommands;

import static frc.lib.util.flippable.Flippable.IS_RED_ALLIANCE_TRIGGER;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.utilities.FieldConstants.*;

public class PathfindingCommands {
    public static void setupFeederPathfinding(Trigger button) {
        final Transform2d distanceToAlignFrom = new Transform2d(new Translation2d(-0.3, 0), Rotation2d.fromDegrees(0));

        for (DriverStation.Alliance alliance : DriverStation.Alliance.values()) {
            for (FeederPosition position : FeederPosition.values()) {
                setupFeederCommand(button, alliance, position, distanceToAlignFrom);
            }
        }
    }

    private static void setupFeederCommand(Trigger button, DriverStation.Alliance alliance, FeederPosition position, Transform2d distanceToAlignFrom) {
        final Pose2d targetPose = getFeederPose(alliance, position);
        final Trigger isInAlignmentZone = isRobotCloserThan(targetPose, 0.8);
        final Trigger isClosestFeeder = getClosestFeederTrigger(position);
        final Trigger isCorrectAlliance = (alliance == DriverStation.Alliance.Red) ? IS_RED_ALLIANCE_TRIGGER : IS_RED_ALLIANCE_TRIGGER.negate();

        button.and(isInAlignmentZone).and(isClosestFeeder).and(isCorrectAlliance)
                .whileTrue(SwerveCommands.goToPosePID(targetPose));

        button.and(isInAlignmentZone.negate()).and(isClosestFeeder).and(isCorrectAlliance)
                .whileTrue(
                        SwerveCommands.goToPoseBezier(targetPose.transformBy(distanceToAlignFrom))
                                .andThen(SwerveCommands.goToPosePID(targetPose))
                );
    }

    private static Pose2d getFeederPose(DriverStation.Alliance alliance, FeederPosition position) {
        if (alliance == DriverStation.Alliance.Blue)
            return position == FeederPosition.TOP ? BLUE_TOP_FEEDER_INTAKE_POSE : BLUE_BOTTOM_FEEDER_INTAKE_POSE;

        return position == FeederPosition.TOP ? RED_TOP_FEEDER_INTAKE_POSE : RED_BOTTOM_FEEDER_INTAKE_POSE;
    }

    private static Trigger getClosestFeederTrigger(FeederPosition position) {
        final Trigger closestFeederIsTop = new Trigger(() -> POSE_ESTIMATOR.getCurrentPose().getY() - FIELD_WIDTH / 2 > 0);
        return position == FeederPosition.TOP ? closestFeederIsTop : closestFeederIsTop.negate();
    }

    private enum FeederPosition {
        TOP, BOTTOM
    }

    private static Trigger isRobotCloserThan(Pose2d targetPose, double allowedDistance) {
        return new Trigger(() -> POSE_ESTIMATOR.getCurrentPose().getTranslation().minus(targetPose.getTranslation()).getNorm() < allowedDistance);
    }
}