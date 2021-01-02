package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.buttons.Button;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;

import lombok.Data;

@Data
public class Intake implements RobotComponent {
    private static final double INTAKE_MODE = 0.239;
    private static final double SHOOTER_MODE = 0.58;

    private DcMotor motor;
    private Servo servo;

    private Button dpadUp = new Button();
    private Button dpadDown = new Button();

    Shooter shooter;

    public Intake(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        motor = hardwareMap.get(DcMotor.class, "intake");
        servo = hardwareMap.get(Servo.class, "intake_servo");

        servo.setPosition(SHOOTER_MODE);
    }

    public void control(Gamepad gamepad) {
        if(gamepad.right_trigger > 0) {
            intake(gamepad.right_trigger);
        } else if(gamepad.left_trigger > 0) {
            outtake(gamepad.left_trigger);
        } else {
            stop();
        }

        dpadUp.input(gamepad.dpad_up);
        dpadDown.input(gamepad.dpad_down);

        if(dpadUp.onPress()) {
            servo.setPosition(servo.getPosition() + 0.005);
        } else if(dpadDown.onPress()) {
            servo.setPosition(servo.getPosition() - 0.005);
        }

        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("Intake Servo", servo.getPosition());
        telemetry.addData("intake Motor", motor.getPower());
        telemetry.addData("r-trigger", gamepad.right_trigger);
        telemetry.addData("l-trigger", gamepad.left_trigger);

    }

    public void intake(double power) {
        motor.setPower(-1 * power);
        shooter.setTargetPosition(Shooter.INTAKE_POS);
        servo.setPosition(INTAKE_MODE);
    }

    public void outtake(double power) {
        motor.setPower(power);
        shooter.setTargetPosition(Shooter.INTAKE_POS);
        servo.setPosition(INTAKE_MODE);
    }

    public void stop() {
        motor.setPower(0);
        servo.setPosition(SHOOTER_MODE);
    }
}
