package frc.robot.commands;

import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.mirrorable.Mirrorable;

import java.util.List;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class PathfindToFeeder {
    public static PathPlannerPath pathfinder() {
        Mirrorable.isRedAlliance();

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                POSE_ESTIMATOR.getCurrentPose(),
                new Pose2d(1.367, 1.367, Rotation2d.fromDegrees(-127.042))
        );

        PathConstraints constraints = new PathConstraints(5,
                3, 2 * Math.PI, 2 * Math.PI);

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(0, Rotation2d.fromDegrees(-127.042)),
                new GoalEndState(0.0, Rotation2d.fromDegrees(-127.042))
        );

        path.preventFlipping = true;
        return path;
    }
}