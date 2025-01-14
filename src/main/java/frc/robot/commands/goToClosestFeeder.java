package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.FlippablePose2d;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.utilities.FieldLocations.BLUE_BOTTOM_FEEDER;
import static frc.robot.utilities.FieldLocations.BLUE_TOP_FEEDER;
import static frc.robot.utilities.PathPlannerConstants.PATHPLANNER_CONSTRAINTS;

public class goToClosestFeeder {
    public static void findClosestFeeder(Trigger button) {
        Trigger closetFeeder = new Trigger(() -> POSE_ESTIMATOR.getCurrentPose().getY() - 4 < 0);

        // flips around both axis instead of only the y
        button.and(closetFeeder).whileTrue(AutoBuilder.pathfindToPose(new FlippablePose2d(BLUE_BOTTOM_FEEDER.toPose2d(),true).get(), PATHPLANNER_CONSTRAINTS));
        button.and(closetFeeder.negate()).whileTrue(AutoBuilder.pathfindToPose(new FlippablePose2d(BLUE_TOP_FEEDER.toPose2d(),true).get(), PATHPLANNER_CONSTRAINTS));
    }
}