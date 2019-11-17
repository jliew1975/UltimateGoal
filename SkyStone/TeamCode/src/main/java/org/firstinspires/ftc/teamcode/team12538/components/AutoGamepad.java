package org.firstinspires.ftc.teamcode.team12538.components;

import org.firstinspires.ftc.teamcode.team12538.detectors.VisionDetector;

public class AutoGamepad {
    public double left_stick_x = 0d;
    public double left_stick_y = 0d;
    public double right_stick_x = 0d;
    public double right_stick_y = 0d;

    public double turnDegree = 0;

    public double timeout = 5d;
    public double distanceInInches = 0d;

    public double power = 0d;

    public VisionDetector detector = null;

    public void reset() {
        left_stick_x = 0d;
        left_stick_y = 0d;
        right_stick_x = 0d;
        right_stick_y = 0d;
        turnDegree = 0;
        power = 0d;
        detector = null;
        timeout = 5d;
    }

    public boolean isStrafing() {
        return left_stick_x != 0;
    }

    public boolean isTurning() {
        return right_stick_x != 0;
    }
}
