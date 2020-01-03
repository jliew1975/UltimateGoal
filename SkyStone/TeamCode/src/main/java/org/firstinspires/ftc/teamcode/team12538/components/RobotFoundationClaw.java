package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class RobotFoundationClaw implements RobotComponent, ControlAware, TelemetryAware {
    public static final double INIT_POSITION = 0d;
    public static final double RAISE_CLAW_POS = 0d;
    public static final double LOWER_CLAW_POS = 1d;

    private Servo leftClaw;
    private Servo rightClaw;
    private volatile ClawMode servoClawMode;

    private Object lock = new Object();
    private volatile boolean busy = false;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        leftClaw = hardwareMap.get(Servo.class, "leftFoundationClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightFoundationClaw");
        rightClaw.setDirection(Servo.Direction.REVERSE);

        leftClaw.setPosition(INIT_POSITION);
        rightClaw.setPosition(INIT_POSITION);

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
        telemetry.addData("leftfoundationClaw", leftClaw.getPosition());
        telemetry.addData("rightfoundationClaw", rightClaw.getPosition());
    }

    public void setClawPosition(double position) {
        leftClaw.setPosition(position);
        rightClaw.setPosition(position);
        if(position > 0) {
            servoClawMode = ClawMode.Open;
        } else {
            servoClawMode = ClawMode.Close;
        }
    }

    public void raiseClaw() {
        OpModeUtils.getGlobalStore().setFoundationClawDown(false);
        leftClaw.setPosition(RAISE_CLAW_POS);
        rightClaw.setPosition(RAISE_CLAW_POS);
        servoClawMode = ClawMode.Open;
    }

    public void lowerClaw() {
        leftClaw.setPosition(LOWER_CLAW_POS);
        rightClaw.setPosition(LOWER_CLAW_POS);
        servoClawMode = ClawMode.Close;
    }
}
