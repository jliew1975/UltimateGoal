package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.buttons.Button;
import org.firstinspires.ftc.teamcode.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.AutonomousColor;
import org.firstinspires.ftc.teamcode.util.GlobalStorage;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.firstinspires.ftc.teamcode.util.ShooterUtils;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

import lombok.Data;

@Data
public class Shooter implements RobotComponent {
    public static final double FIRE = 0d;
    public static final double READY = 1d;

    public static final double INTAKE_POS = 0.470;
    private static final double OFFSET = -0.08;

    private DcMotor motor;
    private Servo trigger;

    private Servo leftServo;
    private Servo rightServo;

    Button dpadUp = new Button();
    Button dpadDown = new Button();

    Button xBtn = new Button();
    Button bBtn = new Button();

    Button leftTrigger = new Button();
    Button rightTrigger = new Button();

    private double power = 1d;

    private volatile int level = 1;

    volatile boolean isBusy = false;

    private Pose2d towerPose;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        motor = hardwareMap.get(DcMotor.class, "shooter");
        trigger = hardwareMap.get(Servo.class, "trigger");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        rightServo.setDirection(Servo.Direction.REVERSE);

        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.FLOAT, motor);
        // MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motor);
        // MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motor);

        // motor.setPositionPIDFCoefficients(0.5);
        // motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.689, 0.1689, 0, 16.89));

        leftServo.setPosition(0.39);
        rightServo.setPosition(0.39 + OFFSET);

        trigger.setPosition(READY);

        if(GlobalStorage.color == AutonomousColor.Blue) {
            towerPose = new Pose2d(72, 36);
        } else {
            towerPose = new Pose2d(72, -36);
        }
    }

    public void control(Gamepad gamepad) {
        if(gamepad.a) {
            trigger.setPosition(FIRE);
        } else {
            trigger.setPosition(READY);
        }

        if(gamepad.right_bumper) {
            start();
        } else {
            stop();
        }

        dpadUp.input(gamepad.dpad_up);
        dpadDown.input(gamepad.dpad_down);

        if(dpadUp.onPress()) {
            leftServo.setPosition(leftServo.getPosition() + 0.01);
            rightServo.setPosition((rightServo.getPosition() + 0.01));
        } else if(dpadDown.onPress()) {
            leftServo.setPosition(leftServo.getPosition() - 0.01);
            rightServo.setPosition((rightServo.getPosition() - 0.01));
        }

        leftTrigger.input(gamepad.left_trigger > 0);
        rightTrigger.input(gamepad.right_trigger > 0);

        if(leftTrigger.onPress()) {
            power += 0.1;
        } else if(rightTrigger.onPress()) {
            power -= 0.1;
        }

        xBtn.input(gamepad.x);
        bBtn.input(gamepad.b);

        if(xBtn.onPress()) {
            lowerShooter();
        } else if(bBtn.onPress()) {
            liftShooter(towerPose);
        }

        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("Right Servo", rightServo.getPosition());
        telemetry.addData("Left Servo", leftServo.getPosition());
        telemetry.addData("HighGoalAngle", Math.toDegrees(ShooterUtils.calculateHighGoalAngle(towerPose)));
        telemetry.addData("ShooterServoAngle", ShooterUtils.getShooterServoAngle(towerPose));
        telemetry.addData("ShooterMotorCalculatedVelocity", ShooterUtils.calculateShooterVelocity(towerPose));
        // telemetry.addData("ShooterMotorActualVelocity", motor.getVelocity());
        telemetry.addData("ShooterMotorActualPower", motor.getPower());
    }

    public void start() {
        motor.setPower(1d);
    }

    public void stop() {
        motor.setPower(0d);
    }

    public void fire() {
        ThreadUtils.getExecutorService().submit(() -> {
            trigger.setPosition(FIRE);
            ThreadUtils.sleep(800);
            trigger.setPosition(READY);
        });
    }

    public void liftShooter(double targetPos) {
        leftServo.setPosition(targetPos);
        rightServo.setPosition(targetPos + OFFSET);
    }

    public void liftShooter(Pose2d targetPose) {
        double servoPos = ShooterUtils.getShooterServoAngle(targetPose);
        leftServo.setPosition(servoPos);
        rightServo.setPosition(servoPos + OFFSET);
        ThreadUtils.sleep(800);
    }

    public void lowerShooter() {
        ThreadUtils.getExecutorService().submit(() -> {
            leftServo.setPosition(0.39);
            rightServo.setPosition(0.39 + OFFSET);
        });
    }

    public void setMode(double mode) {
        leftServo.setPosition(INTAKE_POS);
        rightServo.setPosition(INTAKE_POS + OFFSET);
    }
}
