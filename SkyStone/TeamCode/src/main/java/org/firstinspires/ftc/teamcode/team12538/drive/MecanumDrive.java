package org.firstinspires.ftc.teamcode.team12538.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class MecanumDrive implements TeleOpDrive {
    public enum AutoDirection {Forward, Backward, TurnLeft, TurnRight, StrafeLeft, StrafeRight}

    protected final double WHEEL_COUNTS_PER_REV = 537.6;
    protected final double DEAD_WHEEL_COUNTS_PER_REV = 1600;

    protected final double WHEEL_DIAMETER_INCHES = 4.0;
    protected final double DEAD_WHEEL_DIAMETER_INCHES = 1.96;

    protected final double PI = 3.1415;
    protected final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * PI;
    protected final double DEAD_WHEEL_CIRCUMFERENCE = DEAD_WHEEL_DIAMETER_INCHES * PI;

    protected final double DRIVE_GEAR_REDUCTION = 1.0;  // drive train geared down 1:2

    protected final double WHEEL_ENCODER_TICKS_PER_INCH =
            (WHEEL_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_CIRCUMFERENCE);

    protected final double DEAD_WHEEL_ENCODER_TICKS_PER_INCH =
            (DEAD_WHEEL_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / (DEAD_WHEEL_CIRCUMFERENCE);

    public DcMotorWrapper leftFront = null;
    public DcMotorWrapper rightFront = null;
    public DcMotorWrapper leftRear = null;
    public DcMotorWrapper rightRear = null;

    protected List<DcMotorWrapper> driveMotorList;

    protected List<DcMotorWrapper> strafeEncoderMotors;
    protected List<DcMotorWrapper> directionalEncoderMotors;

    protected Set<String> directionalMotorNames;

    protected BNO055IMUImpl imu = null;

    protected double globalAngle;
    protected Orientation lastAngles = new Orientation();

    protected Telemetry telemetry;
    protected ElapsedTime runtime = new ElapsedTime();

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

        if(OpModeUtils.isResetEncoder()) {
            MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, driveMotorList);
        }

        MotorUtils.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, driveMotorList);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, driveMotorList);

        telemetry = OpModeUtils.getOpMode().telemetry;
    }

    public void init_imu() {
        imu = OpModeUtils.getHardwareMap().get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.loggingEnabled = false;   //For debugging
        imu.initialize(parameters);

        while (OpModeUtils.opModeIsActive() && !imu.isGyroCalibrated()) {
            ThreadUtils.idle();
        }

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("imu", "finish imu calabration");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
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

        double powerV1 = v1 * powerMultiplier;
        double powerV2 = v2 * powerMultiplier;
        double powerV3 = v3 * powerMultiplier;
        double powerV4 = v4 * powerMultiplier;

        if(gamepad.right_bumper) {
            powerV2 = 0d;
            powerV4 = 0d;
        } else if(gamepad.left_bumper) {
            powerV1 = 0d;
            powerV3 = 0d;
        }

        leftFront.setPower(powerV1);
        rightFront.setPower(powerV2);
        leftRear.setPower(powerV3);
        rightRear.setPower(powerV4);
    }

    @Override
    public void resetEncoderValues() {
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, driveMotorList);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, driveMotorList);
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