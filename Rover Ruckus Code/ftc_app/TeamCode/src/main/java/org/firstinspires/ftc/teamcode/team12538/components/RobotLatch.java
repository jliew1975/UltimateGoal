package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class RobotLatch implements RobotMechanic {
    private Servo hook;
    private Servo hangLeg;
    private DcMotor scissorLift;
    private DcMotor scissorLiftHelp;

    private ElapsedTime runtime = new ElapsedTime();

    private volatile boolean scissorLiftBusy = false;
    private volatile boolean scissorLiftUpPosInd = false;

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        hook = hardwareMap.get(Servo.class, "latch");
        hook.setPosition(0.1);

        hangLeg = hardwareMap.get(Servo.class, "hang_leg");
        hangLeg.setPosition(0d);

        scissorLift = hardwareMap.get(DcMotor.class, "lift");
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, scissorLift);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, scissorLift);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorLift);
    }

    public void setHookPosition(double position) {
        hook.setPosition(position);
    }

    public void autoHook() {
        hook.setPosition(1.0);
    }

    public void autoUnhook() {
        hook.setPosition(0.5);
    }

    public void teleHook() {
        hook.setPosition(0.0);
    }

    public void teleUnhook() {
        hook.setPosition(1.0);
    }

    public void adjustHangLegPosition(double position) {
        hangLeg.setPosition(hangLeg.getPosition() + position);
    }

    public boolean shouldLowerSupportLeg() {
        return (scissorLift.getCurrentPosition() > 4500);
    }

    public void autoLegUp() {
        hangLeg.setPosition(0.0);
    }

    public void autoLegDown() {
        hangLeg.setPosition(1.0);
    }

    public void setHangLeg(double position) {
        if(hangLeg.getPosition() < 1 || hangLeg.getPosition() > 0 ) {
            hangLeg.setPosition(hangLeg.getPosition() + position);
        }
    }

    public void powerLift(double power) {
        powerLift(power, -1);
    }

    public void powerLift(double power, int constraint) {
        if(constraint != -1) {
            if(scissorLift.getCurrentPosition() <= constraint) {
                power = 0;
            }
        }

        scissorLift.setPower(power);
    }

    public void powerLiftOnUpPosition(final double power, final int targetPosition) {
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLift);

        double adjustedPower = power;
        int currentPosition = scissorLift.getCurrentPosition();

        scissorLift.setTargetPosition(targetPosition);
        scissorLift.setPower(Math.abs(adjustedPower));

        runtime.reset();
        while (OpModeUtils.opModeIsActive() && scissorLift.isBusy() && runtime.seconds() < 8d) {
            ThreadUtils.idle();
            if(shouldLowerSupportLeg()) {
                autoLegDown();
            }
        }

        scissorLift.setPower(0d);
    }

    public void powerLiftOnDownPosition(final double power, final int targetPosition) {
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLift);

        double adjustedPower = power;
        int currentPosition = scissorLift.getCurrentPosition();

        scissorLift.setTargetPosition(targetPosition);
        scissorLift.setPower(adjustedPower);

        while(OpModeUtils.opModeIsActive() && scissorLift.isBusy()) {
            ThreadUtils.idle();
            if(!shouldLowerSupportLeg()) {
                autoLegUp();
            }
        }

        scissorLift.setPower(0d);
    }

    public void unlatch() {
        // raise scissor lift for landing
        powerLiftOnUpPosition(1d, 7850);
        ThreadUtils.sleep(500);

        // unlatch robot from lander
        autoUnhook();
        ThreadUtils.sleep(500);

        // lower scissor lift
        powerLiftOnDownPosition(1d, 10);
        scissorLift.setPower(0);
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("lift", scissorLift.getCurrentPosition());
        telemetry.addData("hangLeg", hangLeg.getPosition());
    }
}
