package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.function.Supplier;

import static frc.lib.util.flippable.Flippable.IS_RED_ALLIANCE_TRIGGER;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.utilities.FieldConstants.*;

public class feederPathfinding {
    public static void setupFeederPathfinding(Trigger button) {
        Transform2d finalAlignment = new Transform2d(new Translation2d(-0.3,0), Rotation2d.fromDegrees(0));

        final Trigger isInBottomFeederZone = isRobotCloserThan(BLUE_BOTTOM_FEEDER_INTAKE_POSE, 0.8);
        final Trigger isInTopFeederZone = isRobotCloserThan(BLUE_TOP_FEEDER_INTAKE_POSE, 0.8);
        final Trigger closetFeederIsTop = new Trigger(() -> POSE_ESTIMATOR.getCurrentPose().getY() - FIELD_WIDTH / 2 > 0);

        button.and(isInBottomFeederZone).and(closetFeederIsTop.negate()).and(IS_RED_ALLIANCE_TRIGGER.negate())
                .whileTrue(
                SwerveCommands.goToPoseWithPID(BLUE_BOTTOM_FEEDER_INTAKE_POSE));

        button.and(isInBottomFeederZone.negate()).and(closetFeederIsTop.negate()).and(IS_RED_ALLIANCE_TRIGGER.negate()).whileTrue(
                SwerveCommands.goToPoseBezier(BLUE_BOTTOM_FEEDER_INTAKE_POSE.transformBy(finalAlignment), 0)
                        .andThen(
                                SwerveCommands.goToPoseWithPID(BLUE_TOP_FEEDER_INTAKE_POSE)
                        )
        );

        button.and(isInTopFeederZone).and(closetFeederIsTop).and(IS_RED_ALLIANCE_TRIGGER.negate()).whileTrue(
                SwerveCommands.goToPoseWithPID(BLUE_TOP_FEEDER_INTAKE_POSE));

        button.and(isInTopFeederZone.negate()).and(closetFeederIsTop).and(IS_RED_ALLIANCE_TRIGGER.negate()).whileTrue(
                SwerveCommands.goToPoseBezier(BLUE_TOP_FEEDER_INTAKE_POSE.transformBy(finalAlignment), 0)
                        .andThen(
                                SwerveCommands.goToPoseWithPID(BLUE_TOP_FEEDER_INTAKE_POSE)
                        )
        );


        button.and(isInBottomFeederZone).and(closetFeederIsTop.negate()).and(IS_RED_ALLIANCE_TRIGGER)
                .whileTrue(
                        SwerveCommands.goToPoseWithPID(RED_BOTTOM_FEEDER_INTAKE_POSE));

        button.and(isInBottomFeederZone.negate()).and(closetFeederIsTop.negate()).and(IS_RED_ALLIANCE_TRIGGER).whileTrue(
                SwerveCommands.goToPoseBezier(RED_BOTTOM_FEEDER_INTAKE_POSE.transformBy(finalAlignment), 0)
                        .andThen(
                                SwerveCommands.goToPoseWithPID(RED_BOTTOM_FEEDER_INTAKE_POSE)
                        )
        );

        button.and(isInTopFeederZone).and(closetFeederIsTop).and(IS_RED_ALLIANCE_TRIGGER).whileTrue(
                SwerveCommands.goToPoseWithPID(RED_TOP_FEEDER_INTAKE_POSE));

        button.and(isInTopFeederZone.negate()).and(closetFeederIsTop).and(IS_RED_ALLIANCE_TRIGGER).whileTrue(
                SwerveCommands.goToPoseBezier(RED_TOP_FEEDER_INTAKE_POSE.transformBy(finalAlignment), 0)
                        .andThen(
                                SwerveCommands.goToPoseWithPID(RED_TOP_FEEDER_INTAKE_POSE)
                        )
        );
    }

    private static Trigger isRobotCloserThan(Pose2d targetPose, double allowedDistance) {
        return new Trigger(() -> POSE_ESTIMATOR.getCurrentPose().getTranslation().minus(targetPose.getTranslation()).getNorm() < allowedDistance);
    }
}