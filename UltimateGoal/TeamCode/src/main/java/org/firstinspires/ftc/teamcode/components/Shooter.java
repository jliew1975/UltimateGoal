package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.buttons.Button;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

import java.util.HashMap;
import java.util.Map;

import lombok.Data;

@Data
public class Shooter implements RobotComponent {
    public static final double FIRE = 0d;
    public static final double READY = 1d;

    public static final double LIFT = 0.43;
    public static final double REST = 0d;

    private DcMotor motor;
    private Servo trigger;

    private Servo leftServo;
    private Servo rightServo;

    Button dpadUp = new Button();
    Button dpadDown = new Button();

    Button xBtn = new Button();
    Button bBtn = new Button();

    private volatile int level = 1;

    Map<Integer, Double> presetLevel = new HashMap<>();

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        motor = hardwareMap.get(DcMotor.class, "shooter");
        trigger = hardwareMap.get(Servo.class, "trigger");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        rightServo.setDirection(Servo.Direction.REVERSE);

        // leftServo.scaleRange(REST, 0.43);
        // rightServo.scaleRange(REST, 0.43);

        //motor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftServo.setPosition(0d);
        rightServo.setPosition(0d);

        presetLevel.put(1, 0.4100);
        presetLevel.put(2, 0.4100);
        presetLevel.put(3, 0.4250);
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
            if(getPosition() < 0.45) {
                leftServo.setPosition(leftServo.getPosition() + 0.01);
                rightServo.setPosition(rightServo.getPosition() + 0.01);
            }
        } else if(dpadDown.onPress()) {
            if(getPosition() > 0d) {
                leftServo.setPosition(leftServo.getPosition() - 0.01);
                rightServo.setPosition(rightServo.getPosition() - 0.01);
            }
        }

        xBtn.input(gamepad.x);
        bBtn.input(gamepad.b);

        if(xBtn.onPress()) {
            lowerShooter();
        } else if(bBtn.onPress()) {
            leftServo.setPosition(presetLevel.get(level));
            rightServo.setPosition(presetLevel.get(level));

            level++;
            if(level == 4) {
                level = 1;
            }
        }

        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("leftServo", leftServo.getPosition());
        telemetry.addData("rightServo", rightServo.getPosition());
        telemetry.addData("targetLevel", level);
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
            ThreadUtils.sleep(500);
            trigger.setPosition(READY);
        });
    }

    public void liftShooter(int level) {
        leftServo.setPosition(presetLevel.get(level));
        rightServo.setPosition(presetLevel.get(level));
    }

    public void lowerShooter() {
        ThreadUtils.getExecutorService().submit(() -> {
            leftServo.setPosition(0.2);
            rightServo.setPosition(0.2);
            ThreadUtils.sleep(800);
            leftServo.setPosition(0d);
            rightServo.setPosition(0d);
        });
    }

    private double getPosition() {
        return Math.min(leftServo.getPosition(), rightServo.getPosition());
    }
}
