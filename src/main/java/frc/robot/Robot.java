
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.generic.hardware.HardwareManager;
import org.littletonrobotics.junction.LoggedRobot;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.poseestimation.photoncamera.CameraFactory.VISION_SIMULATION;
import static frc.robot.subsystems.leds.Leds.setLEDToPositionIndicator;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private RobotContainer robotContainer;

    private final Field2d simulatedVisionField = VISION_SIMULATION.getDebugField();

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

    }

    @Override
    public void disabledPeriodic() {
        setLEDToPositionIndicator(POSE_ESTIMATOR.getCurrentPose(), new Translation2d(2, 2));
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        HardwareManager.updateSimulation();

        VISION_SIMULATION.updateRobotPose(POSE_ESTIMATOR.getOdometryPose());
        simulatedVisionField.getObject("EstimatedRobot").setPose(POSE_ESTIMATOR.getCurrentPose());
    }

    @Override
    public void close() {
        super.close();
    }
}
