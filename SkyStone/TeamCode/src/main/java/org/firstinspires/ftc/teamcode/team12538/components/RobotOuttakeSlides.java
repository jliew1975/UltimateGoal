package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.Arrays;
import java.util.List;

public class RobotOuttakeSlides implements RobotComponent, TelemetryAware {
    public static final int ENCODER_TICKS_FOR_INTAKE = 900;
    public static final int ENCODER_TICKS_FOR_DEPLOY = 1750;
    public static final int ENCODER_TICKS_FOR_MAX_HEIGHT = 3100;
    public static final int ENCODER_TICKS_FOR_STONE_PICKUP = 20;
    public static final int ENCODER_TICKS_FOR_STONE_DROP = 500;

    private DcMotorWrapper leftOuttakeSlide = null;
    private DcMotorWrapper rightOuttakeSlide = null;

    private List<DcMotorWrapper> motors;


    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        leftOuttakeSlide = new DcMotorWrapper("leftOuttakeSlide", hardwareMap);
        rightOuttakeSlide = new DcMotorWrapper("rightOuttakeSlide", hardwareMap);

        leftOuttakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOuttakeSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = Arrays.asList(leftOuttakeSlide, rightOuttakeSlide);

        if(OpModeUtils.getGlobalStore().runMode == OpModeStore.RunMode.Autonomous) {
            MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motors);
        }

        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, motors);
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
            int targetPosition = position;

            /*
            if(position == ENCODER_TICKS_FOR_INCREMENTAL_STONE_HEIGHT) {
                targetPosition = motor.getCurrentPosition() + ENCODER_TICKS_FOR_INCREMENTAL_STONE_HEIGHT;
            }
            */

            // Prevent slides going beyond max height
            if(targetPosition > ENCODER_TICKS_FOR_MAX_HEIGHT) {
                targetPosition = ENCODER_TICKS_FOR_MAX_HEIGHT;
            }

            motor.setTargetPosition(targetPosition);
        }

        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, motors);
        for(DcMotorWrapper motor : motors) {
            setPower(1d);
        }

        Telemetry telemetry = OpModeUtils.getTelemetry();
        while(OpModeUtils.opModeIsActive() && MotorUtils.motorIsBusy(motors)) {

            telemetry.addData("Path1", "Running to %7d : %7d",
                    motors.get(0).getTargetPosition(), motors.get(0).getTargetPosition());

            telemetry.addData("Path2", "Running at %7d : %7d",
                    motors.get(1).getCurrentPosition(), motors.get(1).getCurrentPosition());
            telemetry.update();
        }

        setPower(0d);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);
    }

    public int getCurrentPosition() {
        return Math.min(leftOuttakeSlide.getCurrentPosition(), rightOuttakeSlide.getCurrentPosition());
    }
}
