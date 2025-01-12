package frc.robot.commands;

import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.mirrorable.Mirrorable;

import java.util.List;

public class PathfindToFeeder {
    public static PathPlannerPath pathfinder() {
        Mirrorable.isRedAlliance();

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(1.367, 1.367, Rotation2d.fromDegrees(-127.042)),
                new Pose2d(3.740, 3.740, Rotation2d.fromDegrees(61.113))
        );

        PathConstraints constraints = new PathConstraints(10.0,
                10.0, 2 * Math.PI, 10 * Math.PI);

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(0, Rotation2d.fromDegrees(-127.042)),
                new GoalEndState(0.0, Rotation2d.fromDegrees(61.113))
        );

        path.preventFlipping = true;
        return path;
    }
}