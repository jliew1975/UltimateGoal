package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.OpModeUtils;

import lombok.Data;

@Data
public class Intake implements RobotComponent {
    private DcMotor motor;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        motor = hardwareMap.get(DcMotor.class, "intake");
    }

    public void control(Gamepad gamepad) {
        if(gamepad.right_trigger > 0) {
            intake(gamepad.right_trigger);
        } else if(gamepad.left_trigger > 0) {
            outtake(gamepad.left_trigger);
        } else {
            stop();
        }
    }

    public void intake(double power) {
        motor.setPower(power);
    }

    public void outtake(double power) {
        motor.setPower(-1 * power);
    }

    public void stop() {
        motor.setPower(0);
    }
}
