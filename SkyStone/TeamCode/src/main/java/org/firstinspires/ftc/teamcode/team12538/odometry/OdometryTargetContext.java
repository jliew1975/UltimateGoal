package org.firstinspires.ftc.teamcode.team12538.odometry;

public class OdometryTargetContext {
    public double targetXPosition = 0d;
    public double targetYPosition = 0d;
    public double robotPower = 0d;
    public double desiredRobotOrientation = 0d;
    public double allowableDistanceError = 0d;

    public void reset() {
        targetXPosition = 0d;
        targetYPosition = 0d;
        robotPower = 0d;
        desiredRobotOrientation = 0d;
        allowableDistanceError = 0d;
    }
}
