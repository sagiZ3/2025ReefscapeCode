package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.json.simple.parser.ParseException;

import java.io.IOException;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.utilities.PathPlannerConstants.PATHPLANNER_CONSTRAINTS;

public class feederPathfinding {
    public static void goToClosestFeeder(Trigger button) {
        final Trigger closetFeeder = new Trigger(() -> POSE_ESTIMATOR.getCurrentPose().getY() - 4 > 0);

        try {
            button.and(closetFeeder).whileTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("nextToTopFeeder").mirrorPath(), PATHPLANNER_CONSTRAINTS));
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Failed to load path", e);
        }
        try {
            button.and(closetFeeder.negate()).whileTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("nextToBottomFeeder").mirrorPath(), PATHPLANNER_CONSTRAINTS));
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Failed to load path", e);
        }
    }
}