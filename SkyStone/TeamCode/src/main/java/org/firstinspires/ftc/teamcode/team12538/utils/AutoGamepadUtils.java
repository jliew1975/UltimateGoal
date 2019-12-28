package org.firstinspires.ftc.teamcode.team12538.utils;

import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;

public class AutoGamepadUtils {
    public static void move(AutoGamepad gamepad,
                            MecanumDrive.AutoDirection direction,
                            double power,
                            double distanceInInchesOrTurnRadian)
    {
        move(gamepad, direction, power, distanceInInchesOrTurnRadian, true);
    }

    public static void move(AutoGamepad gamepad,
                            MecanumDrive.AutoDirection direction,
                            double power,
                            double distanceInInchesOrTurnRadian,
                            boolean reset)
    {
        gamepad.power = power;
        gamepad.direction = direction;
        gamepad.resetAngle = reset;

        switch(direction) {
            case Backward:
                gamepad.left_stick_y = power;
                gamepad.distanceInInches = distanceInInchesOrTurnRadian;
                break;
            case Forward:
                gamepad.left_stick_y = -1 * power;
                gamepad.distanceInInches = distanceInInchesOrTurnRadian;
                break;
            case StrafeLeft:
                gamepad.left_stick_x = -1 * power;
                gamepad.distanceInInches = distanceInInchesOrTurnRadian;
                gamepad.strafing = true;
                break;
            case StrafeRight:
                gamepad.left_stick_x = power;
                gamepad.distanceInInches = distanceInInchesOrTurnRadian;
                gamepad.strafing = true;
                break;
            case CurveLeft:
            case CurveRight:
                if(gamepad.backCurving) {
                    gamepad.left_stick_y = power;
                } else {
                    gamepad.left_stick_y = -1 * power;
                }
                gamepad.distanceInInches = distanceInInchesOrTurnRadian;
                gamepad.curving = true;
                break;
            case TurnLeft:
                gamepad.turnRadian = distanceInInchesOrTurnRadian;
                gamepad.right_stick_x = -1 * power;
                gamepad.turning = true;
                break;
            case TurnRight:
                gamepad.turnRadian = -1 * distanceInInchesOrTurnRadian;
                gamepad.right_stick_x = power;
                gamepad.turning = true;
                break;
        }
    }
}
