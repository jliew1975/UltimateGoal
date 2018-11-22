package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class RobotLatch implements RobotMechanic {
    private Servo hook;
    private Servo hang_leg;
    private DcMotor scissorLift;

    private Object lock = new Object();

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        hook = hardwareMap.get(Servo.class, "latch");
        hang_leg = hardwareMap.get(Servo.class, "hang_leg");
        hook.setPosition(0.0);
        hang_leg.setPosition(0.0);

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
        hook.setPosition(0.0);
    }

    public void teleHook() {
        hook.setPosition(0.0);
    }

    public void teleUnhook() {
        hook.setPosition(1.0);
    }

    public void autoLegUp() { hang_leg.setPosition(1.0); }

    public void autoLegDown() {
        hang_leg.setPosition(0.0);
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

    public void setLiftOnUpPosition() {
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLift);
        scissorLift.setTargetPosition(42000);
        scissorLift.setPower(1);

        while(scissorLift.isBusy()) {
            ThreadUtils.idle();
        }
    }

    public void setLiftOnDownPosition() {
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLift);
        scissorLift.setTargetPosition(0);
        scissorLift.setPower(1);

        while(scissorLift.isBusy()) {
            ThreadUtils.idle();
        }
    }

    public void unlatch() {
        // setLiftOnUpPosition();
        // unhook();
        setLiftOnDownPosition();
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("lift", scissorLift.getCurrentPosition());
    }
}
