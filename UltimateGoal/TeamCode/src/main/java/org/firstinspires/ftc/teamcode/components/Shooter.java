package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

import lombok.Data;

@Data
public class Shooter implements RobotComponent {
    public static final double FIRE = 1d;
    public static final double READY = 0d;

    private DcMotor motor;
    private Servo trigger;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        motor = hardwareMap.get(DcMotor.class, "shooter");
        trigger = hardwareMap.get(Servo.class, "trigger");
    }

    public void control(Gamepad gamepad) {
        if(gamepad.a) {
            trigger.setPosition(FIRE);
        } else {
            trigger.setPosition(READY);
        }
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
            ThreadUtils.sleep(200);
            trigger.setPosition(READY);
        });
    }
}
