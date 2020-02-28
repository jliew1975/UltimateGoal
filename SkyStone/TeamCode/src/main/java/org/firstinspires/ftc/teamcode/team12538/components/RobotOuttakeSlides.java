package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.ext.PIDController;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.Arrays;
import java.util.List;

public class RobotOuttakeSlides implements RobotComponent, TelemetryAware {
    public static final int ENCODER_TICKS_FOR_INTAKE = 300;
    public static final int ENCODER_TICKS_FOR_DEPLOY = 1480;
    public static final int ENCODER_TICKS_FOR_MAX_HEIGHT = 2950;
    public static final int ENCODER_TICKS_FOR_STONE_PICKUP = 50;
    public static final int ENCODER_TICKS_PER_STONE = 500; // 110;
    public static final int ENCODER_TICKS_FOR_STONE_DROP = 500;

    private DcMotorWrapper leftOuttakeSlide = null;
    private DcMotorWrapper rightOuttakeSlide = null;

    private PIDFCoefficients upPIDF = new PIDFCoefficients(10d, 0.049988, 0, 0);
    private PIDFCoefficients downPIDF = new PIDFCoefficients(5d, 0.049988, 0, 0);

    private List<DcMotorWrapper> motors;

    private ElapsedTime runtime = new ElapsedTime();


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
        telemetry.addData("leftOuttakeSlide", leftOuttakeSlide.getMotorAsDcMotorEx().getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());
        telemetry.addData("rightOuttakeSlide", rightOuttakeSlide.getMotorAsDcMotorEx().getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());

    }

    public void setPower(double power) {
        setPower(power, true);
    }

    public void setPower(double power, boolean limit) {
        if(limit) {
            if(power > 0 && getCurrentPosition() > ENCODER_TICKS_FOR_MAX_HEIGHT) {
                power = 0;
            } else if(power < 0 && getCurrentPosition() < 0) {
                power = 0;
            }
        }
        leftOuttakeSlide.setPower(power);
        rightOuttakeSlide.setPower(power);
    }

    public int getEncoderTicksForStoneHeight(int height) {
        int targetPosition = height * ENCODER_TICKS_PER_STONE;
        if(targetPosition > ENCODER_TICKS_FOR_MAX_HEIGHT) {
            targetPosition = ENCODER_TICKS_FOR_MAX_HEIGHT;
        }

        return targetPosition;
    }

    public void runToStoneHeight(int height) {
        int targetPosition = height * ENCODER_TICKS_PER_STONE;
        if(targetPosition > ENCODER_TICKS_FOR_MAX_HEIGHT) {
            targetPosition = ENCODER_TICKS_FOR_MAX_HEIGHT;
        }



        runToPosition(targetPosition);
    }

    public void runToPosition(int position) {
        runToPosition(position, false);
    }

    public void runToPosition(int position, boolean slowDown) {
        for(DcMotorWrapper motor : motors) {
            int targetPosition = position;

            // Prevent slides going beyond max height
            if(targetPosition > ENCODER_TICKS_FOR_MAX_HEIGHT) {
                targetPosition = ENCODER_TICKS_FOR_MAX_HEIGHT;
            } else if(targetPosition < 0) {
                targetPosition = 0;
            }

            motor.setTargetPosition(targetPosition);
        }

        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, motors);
        setPower(slowDown ? 0.5 : 1d, false);

        runtime.reset();
        Telemetry telemetry = OpModeUtils.getTelemetry();
        while(OpModeUtils.opModeIsActive() && MotorUtils.motorIsBusy(motors) && runtime.seconds() < 5) {
            telemetry.addData("Path1", "Running to %7d : %7d",
                    motors.get(0).getTargetPosition(), motors.get(0).getTargetPosition());

            telemetry.addData("Path2", "Running at %7d : %7d",
                    motors.get(1).getCurrentPosition(), motors.get(1).getCurrentPosition());
            telemetry.update();
        }

        setPower(0);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);
    }

    public int getCurrentPosition() {
        return Math.min(leftOuttakeSlide.getCurrentPosition(), rightOuttakeSlide.getCurrentPosition());
    }
}
