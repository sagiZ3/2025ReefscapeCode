package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.generic.PID;
import frc.lib.generic.hardware.pigeon.Pigeon;
import frc.lib.generic.hardware.pigeon.PigeonConfiguration;
import frc.lib.generic.hardware.pigeon.PigeonFactory;
import frc.lib.generic.hardware.pigeon.PigeonSignal;

public class SwerveConstants {
    public static final double DRIVE_GEAR_RATIO = (6.75);
    public static final double STEER_GEAR_RATIO = (150.0 / 7.0);

    public static final double WHEEL_DIAMETER = 0.0930234381868334;

    static final double WHEEL_BASE = 0.565;
    static final double TRACK_WIDTH = 0.615;

    public static final double MAX_SPEED_MPS = 5.1;
    public static final double MAX_ROTATION_RAD_PER_S = 3 * Math.PI;

    public static final double DRIVE_BASE_RADIUS = new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2).getNorm();

    public static RobotConfig PATHPLANNER_ROBOT_CONFIGURATION = null;

    protected static final PPHolonomicDriveController PATHPLANNER_PID_CONSTANTS = new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)
                );


    protected static final Translation2d[] MODULE_LOCATIONS = {
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    };

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(MODULE_LOCATIONS);

    public static final double
            DRIVE_NEUTRAL_DEADBAND = 0.15,
            ROTATION_NEUTRAL_DEADBAND = 0.15;

    static final PID SWERVE_TRANSLATION_CONTROLLER = new PID(5,0,0);
    static final ProfiledPIDController SWERVE_ROTATION_CONTROLLER = new ProfiledPIDController(
            3.9, 0, 0.05,
            new TrapezoidProfile.Constraints(360, 360)
    );


    static final Pigeon GYRO = PigeonFactory.createIMU("GYRO", 30);

    static {
        configureGyro();
        configureRotationController();
        configurePathplannerConfig();
    }

    private static void configureGyro() {
        GYRO.configurePigeon(new PigeonConfiguration());
        GYRO.setupSignalUpdates(PigeonSignal.YAW, true);
    }

    private static void configureRotationController() {
        SWERVE_ROTATION_CONTROLLER.enableContinuousInput(-180, 180);
        SWERVE_ROTATION_CONTROLLER.setTolerance(1.5);
    }

    private static void configurePathplannerConfig() {
        try {
            PATHPLANNER_ROBOT_CONFIGURATION = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
