package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class RobotStoneClaw implements RobotComponent, TelemetryAware {
    public static final double CLAW_CLOSE_POSITION = 0.8;
    public static final double CLAW_OPEN_POSITION = 0.5;
    public static final double CLAW_INTAKE_POSITION = 0d;

    public static final double ARM_DEPLOYMENT_POSITION = 0.9d;
    public static final double ARM_STONE_PICKUP_POSITION = 0d;

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
        stoneClaw.setPosition(CLAW_INTAKE_POSITION);

        setArmPosition(0);
    }

    @Override
    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("stoneClaw", stoneClaw.getPosition());
        telemetry.addData("leftArm", leftArm.getPosition());
        telemetry.addData("rightArm", rightArm.getPosition());
    }

    public void setClawPosition(double position) {
        stoneClaw.setPosition(position);
    }

    public double getArmPosition() {
        return Math.min(leftArm.getPosition(), rightArm.getPosition());
    }

    public void setArmPosition(double position) {
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }

}
