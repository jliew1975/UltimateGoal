package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class RobotStoneClaw implements RobotComponent, TelemetryAware {
    public static final float CLAW_CLAM_POSITION = 1f;
    public static final float CLAW_RELEASE_POSITION = 0f;

    public static final float ARM_DEPLOYMENT_POSITION = 1f;
    public static final float ARM_STONE_PICKUP_POSITION = 0f;

    private Servo leftArm;
    private Servo rightArm;

    private Servo stoneClaw;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();

        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        rightArm.setDirection(Servo.Direction.REVERSE);

        stoneClaw = hardwareMap.get(Servo.class, "stoneClaw");
        stoneClaw.setPosition(CLAW_RELEASE_POSITION);
    }

    @Override
    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("stoneClaw", stoneClaw.getPosition());
        telemetry.addData("leftArm", leftArm.getPosition());
        telemetry.addData("rightArm", rightArm.getPosition());
    }

    public void setClawPosition(float position) {
        stoneClaw.setPosition(position);
    }

    public void setArmPosition(float position) {
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }

}
