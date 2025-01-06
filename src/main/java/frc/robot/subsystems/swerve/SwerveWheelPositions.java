package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveWheelPositions {
    private final SwerveModulePosition[] wheelPositions;

    public SwerveWheelPositions(SwerveModulePosition[] wheelPositions) {
        this.wheelPositions = wheelPositions.clone();
    }

    public SwerveModulePosition[] getWheelPositions() {
        return wheelPositions;
    }
}
