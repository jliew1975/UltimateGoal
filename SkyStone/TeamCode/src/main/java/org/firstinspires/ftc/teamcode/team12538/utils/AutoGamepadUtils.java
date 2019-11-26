package org.firstinspires.ftc.teamcode.team12538.utils;

import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;

public class AutoGamepadUtils {
    public static void move(AutoGamepad gamepad,
                            MecanumDrive.AutoDirection direction,
                            double power,
                            double distanceInInchesOrTurnDegree)
    {
        gamepad.power = power;

        switch(direction) {
            case Backward:
                gamepad.left_stick_y = power;
                gamepad.distanceInInches = distanceInInchesOrTurnDegree;
                break;
            case Forward:
                gamepad.left_stick_y = -1 * power;
                gamepad.distanceInInches = distanceInInchesOrTurnDegree;
                break;
            case StrafeLeft:
                gamepad.left_stick_x = -1 * power;
                gamepad.distanceInInches = distanceInInchesOrTurnDegree;
                break;
            case StrafeRight:
                gamepad.left_stick_x = power;
                gamepad.distanceInInches = distanceInInchesOrTurnDegree;
                break;
            case ConceringLeft:
                gamepad.left_stick_y = -1 * power;
                gamepad.distanceInInches = distanceInInchesOrTurnDegree;
                gamepad.conceringLeft = true;
                break;
            case ConceringRight:
                gamepad.left_stick_y = -1 * power;
                gamepad.distanceInInches = distanceInInchesOrTurnDegree;
                gamepad.conceringRight = true;
                break;
            case TurnLeft:
                gamepad.turnDegree = -1 * distanceInInchesOrTurnDegree;
                gamepad.right_stick_x = -1 * power;
                gamepad.turningLeft = true;
                break;
            case TurnRight:
                gamepad.turnDegree = distanceInInchesOrTurnDegree;
                gamepad.right_stick_x = power;
                gamepad.turningRight = true;
                break;
        }
    }
}
