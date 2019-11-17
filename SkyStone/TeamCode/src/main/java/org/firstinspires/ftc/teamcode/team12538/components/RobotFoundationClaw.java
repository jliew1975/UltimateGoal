package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class RobotFoundationClaw implements RobotComponent, ControlAware, TelemetryAware {
    public static final double INIT_POSITION = 0.88;
    public static final double A_POSITION = 0.9;
    public static final double RAISE_CLAW_POS = 0.4;
    public static final double LOWER_CLAW_POS = 0d;

    private Servo servoClaw;
    private volatile ClawMode servoClawMode;

    private Object lock = new Object();
    private volatile boolean busy = false;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        servoClaw = hardwareMap.get(Servo.class, "foundationClaw");

        if(OpModeUtils.getGlobalStore().runMode == OpModeStore.RunMode.Autonomous) {
            servoClaw.setPosition(INIT_POSITION);
        } else {
            servoClaw.setPosition(LOWER_CLAW_POS);
        }

        servoClawMode = ClawMode.Open;
    }

    @Override
    public void control(Gamepad gamepad) {
        if(gamepad.y) {
            raiseClaw();
        } else if(gamepad.a) {
            lowerClaw();
        }
    }

    @Override
    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("foundationClaw", servoClaw.getPosition());
    }

    public void setClawPosition(double position) {
        servoClaw.setPosition(position);
        if(position > 0) {
            servoClawMode = ClawMode.Open;
        } else {
            servoClawMode = ClawMode.Close;
        }
    }

    private void raiseClaw() {
        OpModeUtils.getGlobalStore().setFoundationClawDown(false);
        servoClaw.setPosition(RAISE_CLAW_POS);
        servoClawMode = ClawMode.Open;
    }

    private void lowerClaw() {
        if(OpModeUtils.getGlobalStore().runMode == OpModeStore.RunMode.TeleOp) {
            OpModeUtils.getGlobalStore().setFoundationClawDown(true);
        }

        servoClaw.setPosition(LOWER_CLAW_POS);
        servoClawMode = ClawMode.Close;
    }
}
