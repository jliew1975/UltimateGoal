package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
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

    private DcMotor scissorLiftMotor1;
    private DcMotor scissorLiftMotor2;

    private ElapsedTime runtime = new ElapsedTime();

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        hook = hardwareMap.get(Servo.class, "latch");
        hook.setPosition(0d);

        hangLeg = hardwareMap.get(Servo.class, "hang_leg");
        hangLeg.setPosition(0d);

        scissorLiftMotor1 = hardwareMap.get(DcMotor.class, "lift_1");
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, scissorLiftMotor1);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, scissorLiftMotor1);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorLiftMotor1);

        scissorLiftMotor2 = hardwareMap.get(DcMotor.class, "lift_2");
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, scissorLiftMotor2);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, scissorLiftMotor2);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorLiftMotor2);
    }

    public void setHookPosition(double position) {
        hook.setPosition(position);
    }

    public void autoHook() {
        hook.setPosition(0d);
    }

    public void autoUnhook() {
        hook.setPosition(0.5);
    }

    public void teleHook() {
        hook.setPosition(1d);
    }

    public void teleUnhook() {
        hook.setPosition(0.5);
    }

    public void adjustHangLegPosition(double position) {
        hangLeg.setPosition(hangLeg.getPosition() + position);
    }

    public boolean shouldLowerSupportLeg() {
        return (scissorLiftMotor1.getCurrentPosition() > 4500 || scissorLiftMotor2.getCurrentPosition() > 4500);
    }

    public void autoLegUp() {
        hangLeg.setPosition(0.0);
    }

    public void autoLegDown() {
        hangLeg.setPosition(1.0);
    }

    public void setHangLeg(double position) {
        if(position > 0d && position < 1.0) {
            if (hangLeg.getPosition() < 1 || hangLeg.getPosition() > 0) {
                hangLeg.setPosition(position);
            }
        }
    }

    public void powerLift(double power) {
        powerLift(power, -1);
    }

    public void powerLift(double power, int constraint) {
        if(constraint != -1) {
            if(scissorLiftMotor1.getCurrentPosition() <= constraint || scissorLiftMotor2.getCurrentPosition() <= constraint) {
                power = 0;
            }
        }

        scissorLiftMotor1.setPower(power);
        scissorLiftMotor2.setPower(power);
    }

    public void powerLiftOnUpPosition(final double power, final int targetPosition) {
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLiftMotor1);
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLiftMotor2);

        double adjustedPower = power;

        scissorLiftMotor1.setTargetPosition(targetPosition);
        scissorLiftMotor2.setTargetPosition(targetPosition);

        scissorLiftMotor1.setPower(Math.abs(adjustedPower));
        scissorLiftMotor2.setPower(Math.abs(adjustedPower));

        runtime.reset();
        while (OpModeUtils.opModeIsActive() && scissorLiftMotor1.isBusy() && scissorLiftMotor2.isBusy() && runtime.seconds() < 8d) {
            if(shouldLowerSupportLeg()) {
                autoLegDown();
            }
        }

        scissorLiftMotor1.setPower(0d);
        scissorLiftMotor2.setPower(0d);
    }

    public void powerLiftOnDownPosition(final double power, final int targetPosition) {
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLiftMotor1);
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLiftMotor2);

        double adjustedPower = power;

        scissorLiftMotor1.setTargetPosition(targetPosition);
        scissorLiftMotor2.setTargetPosition(targetPosition);

        scissorLiftMotor1.setPower(adjustedPower);
        scissorLiftMotor2.setPower(adjustedPower);

        runtime.reset();
        while(OpModeUtils.opModeIsActive() && scissorLiftMotor1.isBusy() && scissorLiftMotor2.isBusy() && runtime.seconds() < 8d) {
            if(!shouldLowerSupportLeg()) {
                autoLegUp();
            }
        }

        scissorLiftMotor1.setPower(0d);
        scissorLiftMotor2.setPower(0d);
    }

    public void unlatch() {
        // raise scissor lift for landing
        powerLiftOnUpPosition(1d, 6600);
        ThreadUtils.sleep(500);

        // unlatch robot from lander
        autoUnhook();
        ThreadUtils.sleep(500);

        // lower scissor lift
        powerLiftOnDownPosition(1d, 0);
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("lift_1", scissorLiftMotor1.getCurrentPosition());
        telemetry.addData("lift_2", scissorLiftMotor2.getCurrentPosition());
        telemetry.addData("hangLeg", hangLeg.getPosition());
    }
}
