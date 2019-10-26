package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class RobotFoundationClaw implements RobotComponent, ControlAware, TelemetryAware {
    private Servo servoClaw;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        servoClaw = hardwareMap.get(Servo.class, "foundationClaw");
        servoClaw.setPosition(1d);
    }

    @Override
    public void control(Gamepad gamepad) {
        if(gamepad.y) {
            raiseClaw();
        } else if(gamepad.x) {
            lowerClaw();
        }
    }

    @Override
    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("foundationClaw", servoClaw.getPosition());
    }

    public void raiseClaw() {
        servoClaw.setPosition(1d);
    }

    public void lowerClaw() {
        servoClaw.setPosition(0d);
    }
}
