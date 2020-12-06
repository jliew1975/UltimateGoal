package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.buttons.Button;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.util.OpModeStore;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

public class WobbleArm implements RobotComponent {
    private static final double SERVO_OPEN = 0d;
    private static final double SERVO_CLOSE = 0.8;

    private enum Mode { Latch, Unlatch }

    private Servo wobbleServo;
    private DcMotorEx wobbleMotor;

    private ElapsedTime runtime = new ElapsedTime();

    private volatile boolean isBusy = false;
    private Button btnY = new Button();
    private Button btnA = new Button();
    private Button btnB = new Button();

    private Button dpadUp = new Button();
    private Button dpadDown = new Button();

    private volatile Mode motorMode = Mode.Latch;
    private volatile Mode servoMode = Mode.Latch;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        wobbleMotor = hardwareMap.get(DcMotorEx.class, "wobbleMotor");

        wobbleServo.setDirection(Servo.Direction.REVERSE);
        wobbleServo.setPosition(SERVO_CLOSE);

        if(OpModeUtils.getGlobalStore().getRunMode() == OpModeStore.RunMode.Autonomous) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setPositionPIDFCoefficients(2.0);
        
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void control(Gamepad gamepad) {
        btnY.input(gamepad.y);
        btnA.input(gamepad.a);

        dpadUp.input(gamepad.dpad_up);
        dpadDown.input(gamepad.dpad_down);

        /*
        if (btnY.onPress()) {
            if (!isBusy) {
                isBusy = true;
                ThreadUtils.getExecutorService().submit(() -> {
                    try {
                        int targetPos;
                        if (motorMode == Mode.Unlatch) {
                            targetPos = -200;
                            latch();
                            motorMode = Mode.Latch;
                        } else {
                            targetPos = 630;
                            unlatch();
                            motorMode = Mode.Unlatch;
                        }

                        ThreadUtils.sleep(500);
                        runToPosition(targetPos);
                    } finally {
                        isBusy = false;
                    }
                });
            }


        }
        */

        if (btnA.onPress()) {
            if (!isBusy && motorMode == Mode.Latch) {
                isBusy = true;
                ThreadUtils.getExecutorService().submit(() -> {
                    try {
                        runToPosition(280);
                        unlatch();
                        motorMode = Mode.Unlatch;
                    } finally {
                        isBusy = false;
                    }
                });
            }
        } else if (!isBusy) {
            if (dpadUp.isPressed()) {
                wobbleMotor.setPower(-0.5);
            } else if (dpadDown.isPressed()) {
                wobbleMotor.setPower(0.5);
            } else {
                wobbleMotor.setPower(0.0);
            }
        }

        if (btnY.onPress()) {
            if(servoMode == Mode.Unlatch) {
                latch();
                servoMode = Mode.Latch;
            } else {
                unlatch();
                servoMode = Mode.Unlatch;
            }
        }

        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("wobbleMode", motorMode);
        telemetry.addData("wobbleMotor", wobbleMotor.getCurrentPosition());
    }

    public void latch() {
        wobbleServo.setPosition(SERVO_CLOSE);
    }

    public void unlatch() {
        wobbleServo.setPosition(SERVO_OPEN);
    }

    public void runToPosition(int encoderTicks) {
        wobbleMotor.setTargetPosition(encoderTicks);
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, wobbleMotor);
        wobbleMotor.setPower(0.3);

        runtime.reset();
        runtime.startTime();
        while(OpModeUtils.opModeIsActive() && MotorUtils.motorIsBusy(wobbleMotor)) {
            // wait for motor to reach target position
            if(runtime.milliseconds() > 2000) {
                break;
            }
        }

        wobbleMotor.setPower(0d);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, wobbleMotor);
    }
}
