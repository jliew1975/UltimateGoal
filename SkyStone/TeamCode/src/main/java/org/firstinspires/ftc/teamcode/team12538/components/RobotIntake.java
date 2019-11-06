package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

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
        if(gamepad.right_bumper) {
            setPower(0.5d);
            OpModeUtils.getGlobalStore().setLiftOuttake(true);
        } else if(gamepad.left_bumper) {
            setPower(-0.5d);
            OpModeUtils.getGlobalStore().setLiftOuttake(true);
        } else {
            setPower(0d);
        }
    }

    @Override
    public void printTelemetry() {
        // intensionally left blank
    }

    public void setPower(double power) {
        setPower(power, false, false);
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
}
