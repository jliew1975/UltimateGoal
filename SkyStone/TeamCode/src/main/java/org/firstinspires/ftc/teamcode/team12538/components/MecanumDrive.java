package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class MecanumDrive implements TeleOpDrive, AutoDrive {
    private DcMotorWrapper leftFront = null;
    private DcMotorWrapper rightFront = null;
    private DcMotorWrapper leftRear = null;
    private DcMotorWrapper rightRear = null;

    Map<String, DcMotor> dcMotorMap = new ConcurrentHashMap<>();

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        leftFront = new DcMotorWrapper("leftFront", hardwareMap);
        rightFront = new DcMotorWrapper("rightFront", hardwareMap);
        leftRear = new DcMotorWrapper("leftRear", hardwareMap);
        rightRear = new DcMotorWrapper("rightRear", hardwareMap);

        dcMotorMap.put(leftFront.getName(), leftFront);
        dcMotorMap.put(rightFront.getName(), rightFront);
        dcMotorMap.put(leftRear.getName(), leftRear);
        dcMotorMap.put(rightRear.getName(), rightRear);
    }


    @Override
    public void navigateWithGamepad(Gamepad gamepad) {
        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y);
        double robotAngle = Math.atan2(gamepad.left_stick_x, gamepad.left_stick_y) - Math.PI / 4;
        double rightX = gamepad.right_stick_x;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        double powerMultiplier = 1.0;
        if(gamepad.right_trigger > 0) {
            powerMultiplier = 0.3;
        }

        double powerV1 = powerMultiplier;
        double powerV2 = powerMultiplier;
        double powerV3 = powerMultiplier;
        double powerV4 = powerMultiplier;

        if(gamepad.right_bumper) {
            powerV2 = 0d;
            powerV4 = 0d;
        } else if(gamepad.left_bumper) {
            powerV1 = 0d;
            powerV3 = 0d;
        }

        leftFront.setPower(powerV1 * Math.signum(v1));
        rightFront.setPower(powerV2 * Math.signum(v2));
        leftRear.setPower(powerV3 * Math.signum(v3));
        rightRear.setPower(powerV4 * Math.signum(v4));
    }

    @Override
    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getOpMode().telemetry;
        telemetry.addData("Left Front Motor", leftFront.getCurrentPosition());
        telemetry.addData("Right Front Motor", rightFront.getCurrentPosition());
        telemetry.addData("Left Rear Motor", leftRear.getCurrentPosition());
        telemetry.addData("Right Rear Motor", rightRear.getCurrentPosition());
    }
}