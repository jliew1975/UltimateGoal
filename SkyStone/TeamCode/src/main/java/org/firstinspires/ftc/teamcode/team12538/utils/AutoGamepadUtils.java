package org.firstinspires.ftc.teamcode.team12538.utils;

import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;

public class AutoGamepadUtils {
    public static void move(AutoGamepad gamepad,
                            MecanumDrive.AutoDirection direction,
                            double power,
                            double distanceInInches)
    {
        gamepad.power = power;
        gamepad.distanceInInches = distanceInInches;

        switch(direction) {
            case Backward:
                gamepad.left_stick_y = power;
                break;
            case Forward:
                gamepad.left_stick_y = -1 * power;
                break;
            case StrafeLeft:
                gamepad.left_stick_x = -1 * power;
                break;
            case StrafeRight:
                gamepad.left_stick_x = power;
                break;
            case TurnLeft:
                gamepad.right_stick_x = -1 * power;
                break;
            case TurnRight:
                gamepad.right_stick_x = power;

        }
    }
}
