package org.firstinspires.ftc.teamcode.team12538.components;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class RobotIntake implements RobotComponent, ControlAware, TelemetryAware {
    private DcMotorWrapper leftRoller;
    private DcMotorWrapper rightRoller;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();

        leftRoller =  new DcMotorWrapper("leftRollerIntake", hardwareMap);
        rightRoller = new DcMotorWrapper("rightRollerIntake", hardwareMap);

        leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, leftRoller, rightRoller);
    }

    @Override
    public void control(Gamepad gamepad) {
        double power = 0.8;

        if(gamepad.right_bumper) {
            setPower(power);
            // OpModeUtils.getGlobalStore().setLiftOuttake(true);
        } else if(gamepad.left_bumper) {
            setPower(-1 * power);
            // OpModeUtils.getGlobalStore().setLiftOuttake(true);
        } else {
            setPower(0d);
        }
    }

    @Override
    public void printTelemetry() {
        /*
        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("leftRoller", leftRoller.getMotor().getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("rightRoller", rightRoller.getMotor().getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        */
    }

    public void setPower(double power) {
        setPower(power, false, false);
    }

    public void setPower(final double power, final long timeout) {
        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                setPower(power);
                ThreadUtils.sleep(timeout);
                setPower(0);
            }
        });
    }

    public void setPower(double power, boolean stopleft, boolean stopRight) {
        if(stopleft) {
            leftRoller.setPower(0d);
        } else {
            leftRoller.setPower(power);
        }

        if(stopRight) {
            rightRoller.setPower(0d);
        } else {
            rightRoller.setPower(power);
        }
    }

    // Computes the current battery voltage
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : OpModeUtils.getHardwareMap().voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
