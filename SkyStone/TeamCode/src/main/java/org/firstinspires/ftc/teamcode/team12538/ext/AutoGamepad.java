package org.firstinspires.ftc.teamcode.team12538.ext;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.team12538.detectors.RobotDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive.AutoDirection;

public class AutoGamepad {
    public AutoDirection direction;

    public double left_stick_x = 0d;
    public double left_stick_y = 0d;
    public double right_stick_x = 0d;
    public double right_stick_y = 0d;

    public double turnRadian = 0;

    public double timeout = 5d;
    public double distanceInInches = 0d;

    public boolean strafing = false;
    public boolean turning = false;
    public boolean curving = false;
    public boolean backCurving = false;
    public boolean diagonal = false;

    public boolean resetAngle = true;
    public boolean stopMotor = true;

    public double power = 0d;
    public RobotDetector detector = null;

    public Orientation offsetAngle = null;

    public void reset() {
        left_stick_x = 0d;
        left_stick_y = 0d;
        right_stick_x = 0d;
        right_stick_y = 0d;
        turnRadian = 0;
        power = 0d;
        timeout = 5d;
        detector = null;
        strafing = false;
        turning = false;
        curving = false;
        backCurving = false;
        diagonal = false;
        resetAngle = true;
    }

    public boolean isStrafing() {
        return strafing;
    }

    public boolean isTurning() {
        return turning;
    }

    public boolean isCurving() {
        return curving;
    }

    public boolean isDiagonal() { return diagonal; }

    public boolean isCurvingLeft() {
        return direction == AutoDirection.CurveLeft;
    }

    public boolean isCurvingRight() {
        return direction == AutoDirection.CurveRight;
    }

    public boolean isDiagonalRight() { return direction == AutoDirection.DiagonalRight; }

    public boolean isDiagonalLeft() { return direction == AutoDirection.DiagonalLeft; }
}
