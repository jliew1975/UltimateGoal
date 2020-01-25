package org.firstinspires.ftc.teamcode.team12538.components;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.team12538.robot.Robot;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.states.ToggleBoolean;

public class RobotCapstone implements RobotComponent, ControlAware, TelemetryAware {
    public static final double UP = 0.70;
    public static final double DOWN = 0d;

    private Servo stoneArm;

    private boolean isLiftSlides = true;
    private ToggleBoolean servoToggle = new ToggleBoolean(false);

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();

        stoneArm =  hardwareMap.get(Servo.class, "capstone");
        stoneArm.setPosition(DOWN);
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
        servoToggle.input(gamepad.b);

        if(servoToggle.output()) {
            RobotOuttake outtake = OpModeUtils.getGlobalStore().getComponent("outtake");

            if(isLiftSlides) {
                outtake.outtakeSlides.runToPosition(200);
                isLiftSlides = false;
            }
            stoneArm.setPosition(UP);
            OpModeUtils.getGlobalStore().setCap(true);
        } else {
            stoneArm.setPosition(DOWN);
            isLiftSlides = true;
        }
    }
}
