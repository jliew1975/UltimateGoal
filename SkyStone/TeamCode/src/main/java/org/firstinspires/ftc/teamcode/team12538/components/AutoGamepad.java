package org.firstinspires.ftc.teamcode.team12538.components;

public class AutoGamepad {
    public double left_stick_x = 0d;
    public double left_stick_y = 0d;
    public double right_stick_x = 0d;
    public double right_stick_y = 0d;

    public double timeout = 5d;
    public double distanceInInches = 0d;

    public double power = 0d;

    public void reset() {
        left_stick_x = 0f;
        left_stick_y = 0f;
        right_stick_x = 0f;
        right_stick_y = 0f;
        power = 0d;
    }

    public boolean isStrafing() {
        return left_stick_x != 0;
    }
}
