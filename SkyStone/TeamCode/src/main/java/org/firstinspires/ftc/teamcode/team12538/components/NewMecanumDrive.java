package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class NewMecanumDrive implements TeleOpDrive, AutoDrive {
    private DcMotorWrapper leftFront = null;
    private DcMotorWrapper rightFront = null;
    private DcMotorWrapper leftRear = null;
    private DcMotorWrapper rightRear = null;

    Map<String, DcMotor> dcMotorMap = new ConcurrentHashMap<>();

    // Declare information about robot
    private static final double TRACKLENGTH = 6;
    private static final double TRACKBASE = 7.52;
    private static final double WHEELRADIUS = 1.969;

    @Override
    public void init() {

        // Do all the initial stuff
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        leftFront = new DcMotorWrapper("leftFront", hardwareMap);
        rightFront = new DcMotorWrapper("rightFront", hardwareMap);
        leftRear = new DcMotorWrapper("leftRear", hardwareMap);
        rightRear = new DcMotorWrapper("rightRear", hardwareMap);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        dcMotorMap.put(leftFront.getName(), leftFront);
        dcMotorMap.put(rightFront.getName(), rightFront);
        dcMotorMap.put(leftRear.getName(), leftRear);
        dcMotorMap.put(rightRear.getName(), rightRear);

        List<DcMotor> motorList = new ArrayList<>(dcMotorMap.values());

        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motorList);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, motorList);
    }

    @Override
    public void navigateWithGamepad(Gamepad gamepad) {

        // Set desired velocities
        double xVelocity = gamepad.left_stick_y;
        double yVelocity = gamepad.left_stick_x;
        double angularVelocity = gamepad.right_stick_x;


        // Convert desired velocities into wheel velocities
        double leftFrontPower = (xVelocity - yVelocity - (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;
        double leftRearPower = (xVelocity + yVelocity - (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;
        double rightRearPower = (xVelocity - yVelocity + (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;
        double rightFrontPower = (xVelocity + yVelocity + (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;

        // Calculate wheel with max power
        double maxLeftPower = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
        double maxRightPower = Math.max(Math.abs(rightFrontPower), Math.abs(rightRearPower));
        double maxPower = Math.max(maxLeftPower, maxRightPower);

        // Calculate factor to scale all wheel powers to make less than 1
        double scaleFactor = 1 / maxPower;

        // Set motors to correct powers
        leftFront.setPower(leftFrontPower * scaleFactor);
        leftRear.setPower(leftRearPower * scaleFactor);
        rightRear.setPower(rightRearPower * scaleFactor);
        rightFront.setPower(rightFrontPower * scaleFactor);


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
