package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class RobotStoneArm implements RobotComponent, ControlAware, TelemetryAware {
    public static final double UP = 1d;
    public static final double DOWN = 0d;

    private Servo stoneArm;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();

        stoneArm =  hardwareMap.get(Servo.class, "stoneArm");
        stoneArm.setPosition(UP);
    }

    public double getPosition() {
        return stoneArm.getPosition();
    }

    public void setPosition(double position) {
        stoneArm.setPosition(position);
    }


    @Override
    public void printTelemetry() {
        // intensionally left blank
    }

    @Override
    public void control(Gamepad gamepad) {
        if(gamepad.x) {
            stoneArm.setPosition(UP);
        } else if(gamepad.b) {
            stoneArm.setPosition(DOWN);
        }
    }
}
