package org.firstinspires.ftc.teamcode.team12538.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test_Robot", group="Linear Opmode")
@Disabled
public class Test_RobotMachanism extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor linearSlide = null;
    private DcMotor intake = null;
    private DcMotor swingingArm = null;

    private Servo leftArm;
    private Servo rightArm;

    private Servo leftRelease;
    private Servo rightRelease;

    private DcMotor lift;

    private volatile boolean isHold = true;
    ExecutorService executorService = Executors.newSingleThreadExecutor();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        rightArm.setDirection(Servo.Direction.REVERSE);

        leftArm.setPosition(0.6);
        leftArm.setPosition(0.6);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, intake);

        swingingArm = hardwareMap.get(DcMotor.class, "swingingArm");
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, swingingArm);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, swingingArm);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, swingingArm);

        // MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, swingingArm);
        leftRelease = hardwareMap.get(Servo.class, "leftRelease");
        rightRelease = hardwareMap.get(Servo.class, "rightRelease");
        rightRelease.setDirection(Servo.Direction.REVERSE);

        leftRelease.setPosition(0.5);
        rightRelease.setPosition(0.5);

        linearSlide = hardwareMap.get(DcMotor.class, "armExt");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, linearSlide);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, linearSlide);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, linearSlide);
        */

        lift = hardwareMap.get(DcMotor.class, "lift");
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lift);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, lift);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, lift);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double power = 0.8;
        boolean isSwingingArmUp = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            if(gamepad1.right_trigger > 0) {
                linearSlide.setPower(power);
            } else if(gamepad1.left_trigger > 0) {
                linearSlide.setPower(-power);
            } else {
                linearSlide.setPower(0);
            }

            if(gamepad1.dpad_left) {
                linearSlide.setPower(power);
            } else if(gamepad1.dpad_right) {
                linearSlide.setPower(-power);
            } else {
                linearSlide.setPower(0);
            }

            if(gamepad2.x) {
                // lower
                leftArm.setPosition(0);
                rightArm.setPosition(0);
                isHold = true;
            } else if(gamepad2.a) {
                // lift prepare
                leftArm.setPosition(0.4);
                rightArm.setPosition(0.4);
            } else if (gamepad2.b){
                // deposit
                leftArm.setPosition(1.0);
                rightArm.setPosition(1.0);
            }

            if(gamepad1.right_bumper) {
                intake.setPower(0.7);
            } else if(gamepad1.left_bumper) {
                intake.setPower(-0.7);
            } else {
                intake.setPower(0);
            }

            if(gamepad2.right_bumper) {
                // swingingArm.setTargetPosition(500);
                // lift
                swingingArm.setPower(0.1);
            } else if(gamepad2.left_bumper){
                // swingingArm.setTargetPosition(0);
                // swingingArm.setPower(0.1);
                // lower
                swingingArm.setPower(-0.1);
            } else {
                swingingArm.setPower(0);
            }


            // only set to 0.5 when swinging arm is at lower position
            if(Math.abs(swingingArm.getCurrentPosition()) > 0) {
                if (gamepad2.right_trigger > 0d) {
                    rightRelease.setPosition(0d);
                } else {
                    rightRelease.setPosition(0.5);
                }

                if (gamepad2.left_trigger > 0d) {
                    leftRelease.setPosition(0d);
                } else {
                    leftRelease.setPosition(0.5);
                }
            } else {
                leftRelease.setPosition(0d);
                rightRelease.setPosition(0d);
            }
            */

            if (gamepad2.dpad_up) {
                lift.setPower(0.6);
            } else if (gamepad2.dpad_down) {
                lift.setPower(-0.6);
            } else {
                lift.setPower(0d);
            }

            // telemetry.addData("armExt", linearSlide.getCurrentPosition());
            // telemetry.addData("swingingArm", swingingArm.getCurrentPosition());
            telemetry.addData( "gamepad2.left_stick_x", gamepad2.left_stick_x);
            telemetry.addData( "gamepad2.left_stick_y", gamepad2.left_stick_y);
            telemetry.addData( "gamepad2.right_stick_x", gamepad2.right_stick_x);
            telemetry.addData( "gamepad2.right_stick_y", gamepad2.right_stick_y);
            telemetry.update();
        }
    }
}