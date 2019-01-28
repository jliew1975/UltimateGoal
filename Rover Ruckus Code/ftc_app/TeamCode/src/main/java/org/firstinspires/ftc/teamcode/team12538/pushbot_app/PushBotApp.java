package org.firstinspires.ftc.teamcode.team12538.pushbot_app;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Pushbot - MLK", group = "Test")
@Disabled
public class PushBotApp extends LinearOpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0) {
                if (gamepad1.left_stick_y < 0) {
                    //forward
                    leftDrive.setPower(gamepad1.left_stick_y);
                    rightDrive.setPower(gamepad1.left_stick_y);
                } else if (gamepad1.left_stick_y > 0) {
                    //backwards
                    leftDrive.setPower(gamepad1.left_stick_y);
                    rightDrive.setPower(gamepad1.left_stick_y);
                } else if (gamepad1.right_stick_x > 0) {
                    //right
                    leftDrive.setPower(gamepad1.right_stick_x);
                    rightDrive.setPower(-gamepad1.right_stick_x);
                } else if (gamepad1.right_stick_x < 0) {
                    //left
                    leftDrive.setPower(gamepad1.right_stick_x);
                    rightDrive.setPower(-gamepad1.right_stick_x);
                }
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
        }
    }
}
