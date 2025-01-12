package frc.lib.generic.simulation;

/**
 * An abstract class to simulate the physics of a motor.
 */
public abstract class GenericPhysicsSimulation {
    protected GenericPhysicsSimulation() {
    }

    public abstract double getCurrent();

    public abstract double getSystemPositionRotations();

    public abstract double getSystemVelocityRotationsPerSecond();

    public abstract double getSystemAccelerationRotationsPerSecondSquared();

    public abstract void setVoltage(double voltage);

    public abstract void updateMotor();
}