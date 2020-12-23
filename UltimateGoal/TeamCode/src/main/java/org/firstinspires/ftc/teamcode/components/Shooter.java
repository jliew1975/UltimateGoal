package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommonOpMode;
import org.firstinspires.ftc.teamcode.buttons.Button;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
@Config
public class Shooter implements RobotComponent {
    public static final double FIRE = 0d;
    public static final double READY = 1d;

    public static final double INTAKE_POS = 0.470;
    private static final double OFFSET = -0.08;

    public static int SHOOTING_INTERVAL_1 = 500;
    public static int SHOOTING_INTERVAL_2 = 200;

    public enum SpinWheelMode { SPIN, STOP }

    private DcMotor motor;
    private Servo trigger;

    private Servo leftServo;
    private Servo rightServo;

    Button dpadUp = new Button();
    Button dpadDown = new Button();

    Button aBtn = new Button();
    Button xBtn = new Button();
    Button bBtn = new Button();

    Button leftTrigger = new Button();
    Button rightTrigger = new Button();

    Button rightBumper = new Button();

    private double power = 1d;

    private volatile int level = 1;
    private volatile SpinWheelMode SPIN_MODE = SpinWheelMode.STOP;

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
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motor);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motor);

        // motor.setPositionPIDFCoefficients(0.5);
        // motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.689, 0.1689, 0, 16.89));

        leftServo.setPosition(0.39);
        rightServo.setPosition(0.39 + OFFSET);

        trigger.setPosition(READY);

        towerPose = TargetConstant.towerPose;
    }

    public void control(Gamepad gamepad) {
        aBtn.input(gamepad.a);

        if(aBtn.onPress()) {
            ThreadUtils.getExecutorService().submit(() -> {
                for(int ringCnt = 0; ringCnt < 3; ringCnt++) {
                    trigger.setPosition(FIRE);
                    ThreadUtils.sleep(ringCnt > 0 ? SHOOTING_INTERVAL_2 : SHOOTING_INTERVAL_1);
                    trigger.setPosition(READY);
                    ThreadUtils.sleep(200);
                }
            });
        }

        rightBumper.input(gamepad.right_bumper);

        if(rightBumper.onPress()) {
            if(SPIN_MODE == SpinWheelMode.STOP) {
                SPIN_MODE = SpinWheelMode.SPIN;
                start();
            } else {
                SPIN_MODE = SpinWheelMode.STOP;
                stop();
            }
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
            ThreadUtils.getExecutorService().submit(() -> {
                SampleMecanumDrive drive = OpModeUtils.getDrive();

                Trajectory trajectory =
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-5, -40, 0))
                                .build();
                drive.followTrajectory(trajectory);

                // OpModeUtils.getDrive().turn(ShooterUtils.calculateHorizontalRobotAngle(TargetConstant.towerPose));
            });

            // liftShooter(towerPose);
            liftShooter(0.442);
        }

        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("Right Servo", rightServo.getPosition());
        telemetry.addData("Left Servo", leftServo.getPosition());
        telemetry.addData("HighGoalAngle", Math.toDegrees(ShooterUtils.calculateHighGoalAngle(towerPose)));
        telemetry.addData("ShooterServoAngle", ShooterUtils.getShooterServoAngle(towerPose));
        telemetry.addData("ShooterMotorCalculatedVelocity", ShooterUtils.calculateShooterVelocity(towerPose));
        telemetry.addData("ShooterHorizontalRobotAngle", Math.toDegrees(ShooterUtils.calculateHorizontalRobotAngle(towerPose)));
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

    public void fireSync() {
        trigger.setPosition(FIRE);
        ThreadUtils.sleep(800);
        trigger.setPosition(READY);
    }

    public void liftShooter(double targetPos) {
        leftServo.setPosition(targetPos);
        rightServo.setPosition(targetPos + OFFSET);
    }

    public void liftShooter(Pose2d targetPose) {
        liftShooter(targetPose, 0.003);
    }

    public void liftShooter(Pose2d targetPose, double offset) {
        double servoPos = ShooterUtils.getShooterServoAngle(targetPose) + offset;
        liftShooter(servoPos);
    }

    public void lowerShooter() {
        ThreadUtils.getExecutorService().submit(() -> {
            leftServo.setPosition(0.39);
            rightServo.setPosition(0.39 + OFFSET);
        });
    }

    public void setMode(double mode) {
        leftServo.setPosition(mode);
        rightServo.setPosition(mode + OFFSET);
    }
}
