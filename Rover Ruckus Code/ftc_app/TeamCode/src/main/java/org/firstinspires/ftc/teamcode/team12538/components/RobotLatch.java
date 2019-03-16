package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    private int latchPosition = 6900;
    private int unlatchPosition = 0;

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        latchServo = hardwareMap.get(CRServo.class, "latch");
        latchServo.setDirection(DcMotorSimple.Direction.REVERSE);
        latchServo.setPower(0d);

        latchOpenSensor = hardwareMap.get(DigitalChannel.class, "latch_open_sensor");
        latchCloseSensor = hardwareMap.get(DigitalChannel.class, "latch_close_sensor");


        scissorLiftMotor = hardwareMap.get(DcMotor.class, "jack");
        scissorLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, scissorLiftMotor);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, scissorLiftMotor);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorLiftMotor);
    }

    public void powerLatch(double power) {
        if (power > 0d && latchCloseSensor.getState()) {
            latchServo.setPower(power);
        } else if(power < 0d && latchOpenSensor.getState()) {
            latchServo.setPower(power);
        } else {
            latchServo.setPower(0d);
        }
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
                int currentPos = scissorLiftMotor.getCurrentPosition();
                if (power > 0) {
                    power = 0;
                }
            }

            scissorLiftMotor.setPower(power);
        }
    }

    public void powerLiftRunToPosition(final double power, final int targetPosition) {
        synchronized (scissorLiftMotor) {
            try {
                scissorLiftBusy = true;
                MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLiftMotor);

                double adjustedPower = power;

                int startPos = scissorLiftMotor.getCurrentPosition();

                scissorLiftMotor.setTargetPosition(targetPosition);
                scissorLiftMotor.setPower(adjustedPower);

                runtime.reset();
                while (OpModeUtils.opModeIsActive() && scissorLiftMotor.isBusy() && runtime.seconds() < 5d) {
                    int curPos = scissorLiftMotor.getCurrentPosition();
                    if(targetPosition > 0 && curPos > (targetPosition - 500)) {
                        scissorLiftMotor.setPower(0.3);
                    } else if(targetPosition == 0 && curPos < (targetPosition + 500)) {
                        scissorLiftMotor.setPower(0.3);
                    }

                    Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
                    telemetry.addData("lift", scissorLiftMotor.getCurrentPosition());
                    telemetry.update();
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
        powerLiftRunToPosition(1d, latchPosition);

        // position latch on the close position
        latchClose();

        // lower scissor lift
        powerLiftRunToPosition(1d, unlatchPosition);
    }

    public void unlatch() {
        // raise scissor lift for landing
        powerLiftRunToPosition(1d, latchPosition);

        // unlatch robot from lander
        latchOpen();

        // lower scissor lift
        powerLiftRunToPosition(1d, unlatchPosition);
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("lift", scissorLiftMotor.getCurrentPosition());
        telemetry.addData("openLimitSwitch", latchOpenSensor.getState() ? "No" : "Yes");
        telemetry.addData("closeLimitSwitch", latchCloseSensor.getState() ? "No" : "Yes");
    }
}
