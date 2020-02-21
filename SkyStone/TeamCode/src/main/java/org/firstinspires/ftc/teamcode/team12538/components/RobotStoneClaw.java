package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class RobotStoneClaw implements RobotComponent, TelemetryAware {
    public static final double CLAW_CLOSE_POSITION = 0.8;
    public static final double CLAW_HOLD_POSITION = 0.6;
    public static final double CLAW_OPEN_POSITION = 0.5;
    public static final double CLAW_INTAKE_POSITION = 0d;

    public static final double ARM_DEPLOYMENT_POSITION = 0;
    public static final double ARM_STONE_PICKUP_POSITION = 1;

    private Servo outtakeArm;
    private Servo stoneClaw;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();

        outtakeArm = hardwareMap.get(Servo.class, "leftArm");

        stoneClaw = hardwareMap.get(Servo.class, "stoneClaw");
        stoneClaw.setPosition(CLAW_INTAKE_POSITION);

        setArmPosition(ARM_STONE_PICKUP_POSITION);
    }

    @Override
    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("stoneClaw", stoneClaw.getPosition());
        telemetry.addData("outtakeArm", outtakeArm.getPosition());
    }

    public double getClawPosition() {
        return stoneClaw.getPosition();
    }

    public void setClawPosition(double position) {
        stoneClaw.setPosition(position);
    }

    public double getArmPosition() {
        return outtakeArm.getPosition();
    }

    public void setArmPosition(double position) {
        outtakeArm.setPosition(position);
    }

}
