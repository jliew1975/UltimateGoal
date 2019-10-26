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
import java.util.Arrays;
import java.util.List;

public class RobotOuttakeSlides implements RobotComponent, ControlAware, TelemetryAware {
    private DcMotorWrapper leftOuttakeSlide = null;
    private DcMotorWrapper rightOuttakeSlide = null;

    private List<DcMotorWrapper> motors = new ArrayList<>();

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        leftOuttakeSlide = new DcMotorWrapper("leftOuttakeSlide", hardwareMap);
        rightOuttakeSlide = new DcMotorWrapper("rightOuttakeSlide", hardwareMap);

        leftOuttakeSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightOuttakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftOuttakeSlide, rightOuttakeSlide);

        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, motors);
    }

    @Override
    public void control(Gamepad gamepad) {
        if(gamepad.dpad_up) {
            setPower(-0.5);
        } else if(gamepad.dpad_down) {
            setPower(0.1);
        } else {
            setPower(0d);
        }
    }

    @Override
    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getOpMode().telemetry;
        telemetry.addData("leftOuttakeSlide", leftOuttakeSlide.getCurrentPosition());
        telemetry.addData("rightOuttakeSlide", rightOuttakeSlide.getCurrentPosition());
    }

    public void setPower(double power) {
        leftOuttakeSlide.setPower(power);
        rightOuttakeSlide.setPower(power);
    }

    public void runToPosition(int position) {

        for(DcMotorWrapper motor : motors) {
            int targetPosition = motor.getCurrentPosition() + position;
            motor.setTargetPosition(targetPosition);
        }

        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, motors);
        for(DcMotorWrapper motor : motors) {
            setPower(1d);
        }

        Telemetry telemetry = OpModeUtils.getTelemetry();
        while(OpModeUtils.opModeIsActive() && MotorUtils.motorIsBusy(motors)) {

            telemetry.addData("Path1", "Running to %7d : %7d",
                    motors.get(0).getTargetPosition(), motors.get(1).getTargetPosition());

            telemetry.addData("Path2", "Running at %7d : %7d",
                    motors.get(0).getCurrentPosition(), motors.get(1).getCurrentPosition());
            telemetry.update();
        }

        setPower(0d);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);
    }
}
