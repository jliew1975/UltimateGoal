package org.firstinspires.ftc.teamcode.team12538.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm;
import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
import edu.spa.ftclib.internal.controller.PIDController;
import edu.spa.ftclib.internal.drivetrain.HeadingableMecanumDrivetrain;
import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor;

public abstract class MacanumDriveBase {
    public enum StrafingDirection { Left, Right }

    protected DcMotor frontLeftDrive = null;
    protected DcMotor frontRightDrive = null;
    protected DcMotor rearLeftDrive = null;
    protected DcMotor rearRightDrive = null;

    protected List<DcMotor> motors = new ArrayList<>();

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_forward_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_forward_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        motors.addAll(Arrays.asList(frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive));

        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, motors);
    }

    public void stop() {
        for(DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public void moveForward(double power) {
        // limit power to 1
        power = limitPower(power);

        for(DcMotor motor : motors) {
            motor.setPower(-1 * power);
        }
    }

    public void moveBackward(double power) {
        // limit power to 1
        power = limitPower(power);

        for(DcMotor motor : motors) {
            motor.setPower(1 * power);
        }
    }

    public void strafeRight(double power) {
        // limit power to 1
        power = limitPower(power);

        frontLeftDrive.setPower(power * 1);
        rearLeftDrive.setPower(power * -1);
        frontRightDrive.setPower(power * -1);
        rearRightDrive.setPower(power * 1);

    }

    public void strafeLeft(double power) {
        // limit power to 1
        power = limitPower(power);

        frontLeftDrive.setPower(power * -1);
        rearLeftDrive.setPower(power * 1);
        frontRightDrive.setPower(power * 1);
        rearRightDrive.setPower(power * -1);
    }

    public void turnRight(double power) {
        frontRightDrive.setPower(power);
        rearRightDrive.setPower(power);
        frontLeftDrive.setPower(-power);
        rearLeftDrive.setPower(-power);
    }

    public void turnLeft(double power) {
        frontRightDrive.setPower(-power);
        rearRightDrive.setPower(-power);
        frontLeftDrive.setPower(power);
        rearLeftDrive.setPower(power);
    }

    protected double limitPower(double power) {
        return Math.min(Math.abs(power), 1);
    }

}
