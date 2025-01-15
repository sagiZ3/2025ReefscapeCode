package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.Flippable;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.utilities.FieldLocations.FIELD_WIDTH;
import static frc.robot.utilities.PathPlannerConstants.*;

public class feederPathfinding {
    public static void goToClosestFeeder(Trigger button) {
        final Trigger closetFeeder = new Trigger(() -> POSE_ESTIMATOR.getCurrentPose().getY() - FIELD_WIDTH / 2 > 0);
        final Trigger isRed = new Trigger(Flippable::isRedAlliance);

        button.and(closetFeeder).and(isRed).whileTrue(AutoBuilder.pathfindThenFollowPath(ALIGN_TO_TOP_FEEDER.mirrorPath(), PATHPLANNER_CONSTRAINTS));
        button.and(closetFeeder.negate()).and(isRed).whileTrue(AutoBuilder.pathfindThenFollowPath(ALIGN_TO_BOTTOM_FEEDER.mirrorPath(), PATHPLANNER_CONSTRAINTS));

        button.and(closetFeeder).and(isRed.negate()).whileTrue(AutoBuilder.pathfindThenFollowPath(ALIGN_TO_TOP_FEEDER, PATHPLANNER_CONSTRAINTS));
        button.and(closetFeeder.negate()).and(isRed.negate()).whileTrue(AutoBuilder.pathfindThenFollowPath(ALIGN_TO_BOTTOM_FEEDER, PATHPLANNER_CONSTRAINTS));

    }
}