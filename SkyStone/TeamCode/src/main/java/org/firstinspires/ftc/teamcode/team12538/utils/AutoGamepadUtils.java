package org.firstinspires.ftc.teamcode.team12538.utils;

import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;

public class AutoGamepadUtils {
    public static void move(AutoGamepad gamepad,
                            MecanumDrive.AutoDirection direction,
                            double speed,
                            double distanceInInches)
    {
        gamepad.distanceInInches = distanceInInches;

        switch(direction) {
            case Backward:
                gamepad.left_stick_y = speed;
                break;
            case Forward:
                gamepad.left_stick_y = -1 * speed;
                break;
            case StrafeLeft:
                gamepad.right_stick_x = -1 * speed;
                break;
            case StrafeRight:
                gamepad.right_stick_x = speed;
                break;
            case TurnLeft:
                gamepad.right_stick_x = -1 * speed;
                break;
            case TurnRight:
                gamepad.right_stick_x = speed;

        }
    }
}
