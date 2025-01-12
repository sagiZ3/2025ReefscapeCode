package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.generic.hardware.HardwareManager;
import org.littletonrobotics.junction.LoggedRobot;

import static frc.robot.RobotContainer.LEDS;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.poseestimation.photoncamera.CameraFactory.VISION_SIMULATION;

public class Robot extends LoggedRobot {
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        HardwareManager.initialize(this);
    }

    @Override
    public void robotPeriodic() {
        HardwareManager.update();
        commandScheduler.run();

        POSE_ESTIMATOR.periodic();
    }

    @Override
    public void disabledInit() {
        LEDS.setLEDToPositionIndicator(
                POSE_ESTIMATOR.getCurrentPose().getTranslation(),
                new Translation2d(2, 2),
                10000).schedule();
    }

    @Override
    public void autonomousInit() {
        final Command autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
            autonomousCommand.schedule();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        HardwareManager.updateSimulation();
        VISION_SIMULATION.updateRobotPose(POSE_ESTIMATOR.getOdometryPose());
    }
}
