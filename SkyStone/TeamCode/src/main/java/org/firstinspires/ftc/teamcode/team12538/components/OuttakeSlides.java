package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class OuttakeSlides implements RobotComponent {
    protected DcMotorWrapper leftOuttakeSlide = null;
    protected DcMotorWrapper rightOuttakeSlide = null;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getOpMode().hardwareMap;
        leftOuttakeSlide = new DcMotorWrapper("leftOuttakeSlide", hardwareMap);
        rightOuttakeSlide = new DcMotorWrapper("rightOuttakeSlide", hardwareMap);

        leftOuttakeSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightOuttakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        List<DcMotorWrapper> motors = Arrays.asList(leftOuttakeSlide, rightOuttakeSlide);

        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, motors);
    }

    public void control(Gamepad gamepad) {
        if(gamepad.dpad_up) {
            leftOuttakeSlide.setPower(0.1);
            rightOuttakeSlide.setPower(0.1);
        } else if(gamepad.dpad_down) {
            leftOuttakeSlide.setPower(-0.1);
            rightOuttakeSlide.setPower(-0.1);
        } else {
            leftOuttakeSlide.setPower(0d);
            rightOuttakeSlide.setPower(0d);
        }
    }

    @Override
    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getOpMode().telemetry;
        telemetry.addData("leftOuttakeSlide", leftOuttakeSlide.getCurrentPosition());
        telemetry.addData("rightOuttakeSlide", rightOuttakeSlide.getCurrentPosition());
    }
}
