package org.firstinspires.ftc.teamcode.team12538.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotTest;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Test App", group="Linear Opmode")
@Disabled
public class TestApp extends LinearOpMode {
    enum LiftMode { RunToPosition, Encoder }
    private ElapsedTime runtime = new ElapsedTime();

    private Object lock = new Object();
    private volatile boolean busy = false;

    DcMotor armExtension = null;

    DcMotor depoLift = null;
    Servo depo = null;

    Servo leftArm = null;
    Servo rightArm = null;

    CRServo intake = null;

    Servo phoneTilt = null;
    Servo hook = null;

    private DcMotor scissorLiftMotor1;
    private DcMotor scissorLiftMotor2;

    private Servo hangLeg;

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            OpModeUtils.init(this);
            OpModeUtils.getGlobalStore().setDisableInitPos(true);

            depoLift = hardwareMap.get(DcMotor.class, "depo_lift");
            depoLift.setDirection(DcMotorSimple.Direction.REVERSE);
            MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, depoLift);
            MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, depoLift);
            MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, depoLift);

            armExtension = hardwareMap.get(DcMotor.class, "extend");
            if(OpModeUtils.getGlobalStore().isResetArmExtensionEncoderValue()) {
                MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, armExtension);
            }

            MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, armExtension);
            MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, armExtension);

            depo = hardwareMap.get(Servo.class, "depo");
            depo.setDirection(Servo.Direction.REVERSE);
            depo.setPosition((0d));

            Servo leftArm = hardwareMap.servo.get("l_flip");
            Servo rightArm = hardwareMap.servo.get("r_flip");

            rightArm.setDirection(Servo.Direction.REVERSE);

            leftArm.setPosition(0.2);
            rightArm.setPosition(0.2);

            LiftMode mode = LiftMode.Encoder;

            phoneTilt = phoneTilt = hardwareMap.get(Servo.class, "phone_tilt");
            phoneTilt.setPosition(0.7);

            hook = hardwareMap.get(Servo.class, "latch");
            hook.setPosition(0.1);

            scissorLiftMotor1 = hardwareMap.get(DcMotor.class, "lift_1");
            MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, scissorLiftMotor1);
            MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, scissorLiftMotor1);
            MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorLiftMotor1);

            scissorLiftMotor2 = hardwareMap.get(DcMotor.class, "lift_2");
            MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, scissorLiftMotor2);
            MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, scissorLiftMotor2);
            MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorLiftMotor2);

            hangLeg = hardwareMap.get(Servo.class, "hang_leg");
            hangLeg.setPosition(0d);

            intake = hardwareMap.get(CRServo.class, "intake");
            intake.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();

            while (opModeIsActive()) {
                if(gamepad1.left_bumper) {
                    intake.setPower(1.0);
                } else if(gamepad1.right_bumper) {
                    intake.setPower(-0.1);
                } else {
                    intake.setPower(0);
                }

                armExtension.setPower(gamepad1.right_stick_x);

                if (gamepad1.dpad_up) {
                    MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, depoLift);
                    depoLift.setTargetPosition(3030);
                    depoLift.setPower(1);
                } else if (gamepad1.dpad_down) {
                    MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, depoLift);
                    depoLift.setTargetPosition(0);
                    depoLift.setPower(-1);
                }

                if(gamepad1.a) {
                    depo.setPosition(0.8);
                } else {
                    depo.setPosition(0d);
                }

                if(gamepad1.x) {
                    leftArm.setPosition(0.6);
                    rightArm.setPosition(0.6);
                } else if(gamepad1.b) {
                    leftArm.setPosition(1d);
                    rightArm.setPosition(1d);
                } else if (gamepad1.y) {
                    leftArm.setPosition(0.2);
                    rightArm.setPosition(0.2);
                }

                if(gamepad2.x) {
                    // unhook
                    hook.setPosition(0.5);
                } else if(gamepad2.y) {
                    // tele hook
                    hook.setPosition(1d);
                } else if(gamepad2.a) {
                    // auto hook
                    hook.setPosition(0d);
                }

                if (gamepad2.dpad_up) {
                    powerLift(1.0);
                    if(shouldLowerSupportLeg()) {
                        autoLegDown();
                    } else {
                        autoLegUp();
                    }
                } else if (gamepad2.dpad_down) {
                    powerLift(-1.0);
                    if(shouldLowerSupportLeg()) {
                        autoLegDown();
                    } else {
                        autoLegUp();
                    }
                } else {
                   powerLift(0d);
                }

                telemetry.addData("Lift Mode", mode);
                telemetry.addData("lift_1", scissorLiftMotor1.getCurrentPosition());
                telemetry.addData("lift_2", scissorLiftMotor2.getCurrentPosition());
                telemetry.update();
            }
        } finally {

        }
    }

    public void powerLift(double power) {
        scissorLiftMotor1.setPower(power);
        scissorLiftMotor2.setPower(power);
    }

    public boolean shouldLowerSupportLeg() {
        return (scissorLiftMotor1.getCurrentPosition() > 4500 || scissorLiftMotor2.getCurrentPosition() > 4500);
    }

    public void autoLegUp() {
        hangLeg.setPosition(0.0);
    }

    public void autoLegDown() {
        hangLeg.setPosition(1.0);
    }
}
