package org.firstinspires.ftc.teamcode.team12538.pushbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class PushBot {
    DcMotor leftDrive;
    DcMotor rightDrive;

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        stop();
    }

    public void pivotLeft() {
        rightDrive.setPower(0.1);
        leftDrive.setPower(0);
    }

    public void pivotRight() {
        leftDrive.setPower(0.1);
        rightDrive.setPower(0);
    }

    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void goForward() {
        leftDrive.setPower(0.2);
        rightDrive.setPower(0.2);
    }

    public void goBackward() {
        leftDrive.setPower(-0.2);
        rightDrive.setPower(-0.2);
    }
}
