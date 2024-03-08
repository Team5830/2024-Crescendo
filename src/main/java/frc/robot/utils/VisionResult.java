package frc.robot.utils;

public final class VisionResult {
    public final boolean hasTarget;
    public final double forwardSpeed;
    public final double rotationSpeed;

    // No targets route
    public VisionResult() {
        this.hasTarget = false;
        this.forwardSpeed = 0;
        this.rotationSpeed = 0;
    }

    // With targets route
    public VisionResult(double forwardSpeed, double rotationSpeed) {
        this.hasTarget = true;
        this.forwardSpeed = forwardSpeed;
        this.rotationSpeed = rotationSpeed;
    }
}