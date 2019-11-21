package org.firstinspires.ftc.teamcode.team12538.components;

import org.firstinspires.ftc.teamcode.team12538.detectors.RobotDetector;

public class AutoGamepad {
    public double left_stick_x = 0d;
    public double left_stick_y = 0d;
    public double right_stick_x = 0d;
    public double right_stick_y = 0d;

    public double turnDegree = 0;

    public double timeout = 5d;
    public double distanceInInches = 0d;

    public boolean conceringLeft = false;
    public boolean conceringRight = false;

    public double power = 0d;

    public RobotDetector detector = null;

    public void reset() {
        left_stick_x = 0d;
        left_stick_y = 0d;
        right_stick_x = 0d;
        right_stick_y = 0d;
        turnDegree = 0;
        power = 0d;
        timeout = 5d;
        detector = null;
        conceringLeft = false;
        conceringRight = false;
    }

    public boolean isStrafing() {
        return left_stick_x != 0d && left_stick_y == 0d;
    }

    public boolean isTurning() {
        return left_stick_x == 0 && right_stick_x != 0;
    }

    public boolean isCornering() {
        return conceringLeft || conceringRight;
    }
}
