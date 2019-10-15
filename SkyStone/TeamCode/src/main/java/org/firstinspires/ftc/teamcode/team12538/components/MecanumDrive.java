package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.dogecv.VisionDetector;
import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

public class MecanumDrive implements TeleOpDrive, AutoDrive {
    public enum StrafingDirection { Left, Right }

    private final double WHEEL_COUNTS_PER_REV = 537.6;
    private final double DEAD_WHEEL_COUNTS_PER_REV = 1600;

    private final double WHEEL_DIAMETER_INCHES = 3.93701;
    private final double DEAD_WHEEL_DIAMETER_INCHES = 2.0;

    private final double PI = 3.1415;
    private final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * PI;
    private final double DEAD_WHEEL_CIRCUMFERENCE = DEAD_WHEEL_DIAMETER_INCHES * PI;

    private final double DRIVE_GEAR_REDUCTION = 1.0;  // drive train geared down 1:2

    private final double WHEEL_ENCODER_TICKS_PER_INCH =
            (WHEEL_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_CIRCUMFERENCE);

    private final double DEAD_WHEEL_ENCODER_TICKS_PER_INCH =
            (DEAD_WHEEL_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / (DEAD_WHEEL_CIRCUMFERENCE);

    protected DcMotorWrapper leftFront = null;
    protected DcMotorWrapper rightFront = null;
    protected DcMotorWrapper leftRear = null;
    protected DcMotorWrapper rightRear = null;

    protected List<DcMotorWrapper> driveMotorList;

    protected List<DcMotorWrapper> strafeEncoderMotors;
    protected List<DcMotorWrapper> directionalEncoderMotors;

    protected Set<String> directionalMotorNames;

    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        // Do all the initial stuff
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        leftFront = new DcMotorWrapper("leftFront", hardwareMap);
        rightFront = new DcMotorWrapper("rightFront", hardwareMap);
        leftRear = new DcMotorWrapper("leftRear", hardwareMap);
        rightRear = new DcMotorWrapper("rightRear", hardwareMap);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        driveMotorList = Arrays.asList(leftFront, rightFront, leftRear, rightRear);
        strafeEncoderMotors = Arrays.asList(leftRear);
        directionalEncoderMotors = Arrays.asList(leftFront, rightFront);
        directionalMotorNames = new HashSet<>(Arrays.asList(leftFront.getName(), rightFront.getName()));

        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, driveMotorList);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, driveMotorList);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, driveMotorList);

        telemetry = OpModeUtils.getOpMode().telemetry;
    }

    // TeleOp Drive APIs

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

    // Autonomous Drive APIs

    @Override
    public void resetEncoderValues() {
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, driveMotorList);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, driveMotorList);
    }

    @Override
    public void stop() {
        for(DcMotor motor : driveMotorList) {
            motor.setPower(0d);
        }
    }

    @Override
    public void encoderDrive(double speed, double distanceInInches, double timeout) {
        encoderDrive(speed,distanceInInches, timeout, null);
    }

    @Override
    public void encoderDrive(double speed, double distanceInInches, double timeout, VisionDetector detector) {
        int[] targetPositions = new int[driveMotorList.size()];

        //ensure that the opmode is still active
        if(OpModeUtils.getOpMode().opModeIsActive()) {
            //determine target positions
            int index = 0;

            MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, driveMotorList);

            for(DcMotorWrapper motor : driveMotorList) {
                if(directionalMotorNames.contains(motor.getName())) {
                    targetPositions[index] = motor.getCurrentPosition() - (int) (distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
                    motor.setTargetPosition(targetPositions[index]);
                } else {
                    targetPositions[index] = motor.getCurrentPosition() + (int) (distanceInInches * WHEEL_ENCODER_TICKS_PER_INCH);
                    motor.setTargetPosition(targetPositions[index]);
                }
                index++;
            }

            runtime.reset();

            for(DcMotorWrapper motor : driveMotorList) {
                motor.setPower(Math.abs(speed));
            }

            // keep looping until at least one of the motors finished its movement or timeout is reached
            while(OpModeUtils.opModeIsActive() && (runtime.seconds() < timeout) &&
                    (MotorUtils.motorIsBusy(driveMotorList)))
            {
                if(detector != null) {
                    if(detector.isAligned()) {
                        break;
                    }
                }

                // report target and current positions to driver station
                telemetry.addData("Target", "Running to %7d : %7d : %7d : %7d",
                        targetPositions[0], targetPositions[1], targetPositions[2], targetPositions[3]);

                telemetry.addData("Current", "Running at %7d : %7d : %7d : %7d",
                        driveMotorList.get(0).getCurrentPosition(),
                        driveMotorList.get(1).getCurrentPosition(),
                        driveMotorList.get(2).getCurrentPosition(),
                        driveMotorList.get(3).getCurrentPosition());

                telemetry.update();
            }

            stop();
            MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, directionalEncoderMotors);
        }
    }

    @Override
    public void encoderStrafe(StrafingDirection direction, double speed, double distance, double timeout) {
        encoderStrafe(direction, speed, distance, timeout, null);
    }

    @Override
    public void encoderStrafe(StrafingDirection direction, double speed, double distance, double timeout, VisionDetector detector) {
        // reset encoders
        // MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, driveMotorList);

        double newDistance = (direction == StrafingDirection.Right) ? distance : -1 * distance;

        // ensure that the opmode is still active
        if(OpModeUtils.getOpMode().opModeIsActive()) {
            int directionMultiplier = (direction == StrafingDirection.Left) ? 1 : -1;
            int targetPosition = rightRear.getCurrentPosition() +
                    (directionMultiplier * (int) (newDistance * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));

            runtime.reset();
            for(DcMotorWrapper motor : driveMotorList) {
                if(direction == StrafingDirection.Left) {
                    leftFront.setPower(speed);
                    rightFront.setPower(-speed);
                    leftRear.setPower(speed);
                    rightRear.setPower(-speed);
                } else {
                    leftFront.setPower(-speed);
                    rightFront.setPower(speed);
                    leftRear.setPower(-speed);
                    rightRear.setPower(speed);
                }
            }

            // keep looping until one of the motors finished its movement
            while (OpModeUtils.getOpMode().opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (rightRear.getCurrentPosition() < targetPosition))
            {
                if(detector != null) {
                    if(detector.isAligned()) {
                        break;
                    }
                }

                //report current and target positions to driver station
                telemetry.addData("Target", "Running to %7d", targetPosition);
                telemetry.addData("Current", "Running at %7d", rightRear.getCurrentPosition());
                telemetry.update();
            }

            stop();
        }
    }

    // Telemetry API

    @Override
    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getOpMode().telemetry;
        telemetry.addData("Left Front Motor", leftFront.getCurrentPosition());
        telemetry.addData("Right Front Motor", rightFront.getCurrentPosition());
        telemetry.addData("Left Rear Motor", leftRear.getCurrentPosition());
        telemetry.addData("Right Rear Motor", rightRear.getCurrentPosition());
    }
}