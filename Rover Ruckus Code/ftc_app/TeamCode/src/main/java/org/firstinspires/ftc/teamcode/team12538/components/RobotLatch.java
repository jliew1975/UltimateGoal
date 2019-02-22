package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

import lombok.Data;

@Data
public class RobotLatch implements RobotMechanic {
    private CRServo latchServo;
    private DigitalChannel latchOpenSensor = null;
    private DigitalChannel latchCloseSensor = null;


    private DcMotor scissorLiftMotor;
    private volatile boolean scissorLiftBusy = false;

    private ElapsedTime runtime = new ElapsedTime();

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        // TODO: need to figure out servo direction
        latchServo = hardwareMap.get(CRServo.class, "latch");
        latchServo.setDirection(DcMotorSimple.Direction.REVERSE);
        latchServo.setPower(0d);

        latchOpenSensor = hardwareMap.get(DigitalChannel.class, "latch_open_sensor");
        latchCloseSensor = hardwareMap.get(DigitalChannel.class, "latch_close_sensor");


        scissorLiftMotor = hardwareMap.get(DcMotor.class, "jack");
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, scissorLiftMotor);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, scissorLiftMotor);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorLiftMotor);
    }

    public void latchClose() {
        if(latchCloseSensor.getState()) {
            latchServo.setPower(1d);

            runtime.reset();
            while (OpModeUtils.opModeIsActive() && latchCloseSensor.getState()) {
                // wait until limit sensor tells the latch to stop
            }

            latchServo.setPower(0d);
        }
    }

    public void latchOpen() {
        if(latchOpenSensor.getState()) {
            latchServo.setPower(-1d);

            runtime.reset();
            while (OpModeUtils.opModeIsActive() && latchOpenSensor.getState()) {
                // wait until limit sensor tells the latch to stop
            }

            latchServo.setPower(0d);
        }
    }

    public void powerLift(double power) {
        powerLift(power, -1);
    }

    public void powerLift(double power, int constraint) {
        synchronized (scissorLiftMotor) {
            if(scissorLiftBusy) {
                return;
            }

            if (constraint != -1) {
                if (Math.abs(scissorLiftMotor.getCurrentPosition()) <= constraint) {
                    power = 0;
                }
            }

            scissorLiftMotor.setPower(power);
        }
    }

    public void powerLiftOnUpPosition(final double power, final int targetPosition) {
        synchronized (scissorLiftMotor) {
            try {
                scissorLiftBusy = true;
                MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLiftMotor);

                double adjustedPower = power;

                scissorLiftMotor.setTargetPosition(targetPosition);
                scissorLiftMotor.setPower(-1);

                runtime.reset();
                while (OpModeUtils.opModeIsActive() && scissorLiftMotor.isBusy() && runtime.seconds() < 5d) {
                    // fail-safe logic
                    if(Math.abs(scissorLiftMotor.getCurrentPosition()) >= 8000) {
                        break;
                    }
                }

                scissorLiftMotor.setPower(0d);
                MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorLiftMotor);
            } finally {
                scissorLiftBusy = false;
            }
        }
    }

    public void powerLiftOnDownPosition(final double power, final int targetPosition) {
        synchronized (scissorLiftMotor) {
            try {
                scissorLiftBusy = true;
                MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLiftMotor);

                double adjustedPower = power;

                scissorLiftMotor.setTargetPosition(targetPosition);
                scissorLiftMotor.setPower(adjustedPower);

                runtime.reset();
                while (OpModeUtils.opModeIsActive() && scissorLiftMotor.isBusy() && runtime.seconds() < 5d) {
                    // intentionally left blank for no-op
                }

                scissorLiftMotor.setPower(0d);
                MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorLiftMotor);
            } finally {
                scissorLiftBusy = false;
            }
        }
    }

    public void latch() {
        // raise scissor lift for latching
        // powerLiftOnUpPosition(1d, 7050);
        powerLiftOnUpPosition(1d, 2000);
        ThreadUtils.sleep(500);

        // position latch on the close position
        latchClose();
        ThreadUtils.sleep(500);

        // lower scissor lift
        powerLiftOnDownPosition(1d, 0);
    }

    public void unlatch() {
        latchClose();

        // raise scissor lift for landing
        powerLiftOnUpPosition(1d, 7050);
        ThreadUtils.sleep(500);

        // unlatch robot from lander
        latchOpen();
        ThreadUtils.sleep(500);

        // lower scissor lift
        powerLiftOnDownPosition(1d, 0);
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("lift", scissorLiftMotor.getCurrentPosition());
        telemetry.addData("openLimitSwitch", latchOpenSensor.getState() ? "No" : "Yes");
        telemetry.addData("closeLimitSwitch", latchCloseSensor.getState() ? "No" : "Yes");
    }
}
