package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    private static final double SERVO_OPEN = 0.1;
    private static final double SERVO_CLOSE = 0.5;

    private enum Mode { Latch, Unlatch }

    private Servo wobbleServo;
    private DcMotor wobbleMotor;

    private ElapsedTime runtime = new ElapsedTime();

    private volatile boolean isBusy = false;
    private Button btnY = new Button();
    private Button btnA = new Button();

    private Button dpadUp = new Button();
    private Button dpadDown = new Button();

    private volatile Mode mode = Mode.Latch;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        wobbleMotor = hardwareMap.get(DcMotor.class, "wobbleMotor");

        wobbleServo.setPosition(SERVO_CLOSE);
        wobbleServo.setDirection(Servo.Direction.REVERSE);

        if(OpModeUtils.getGlobalStore().getRunMode() == OpModeStore.RunMode.Autonomous) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void control(Gamepad gamepad) {
        btnY.input(gamepad.y);
        btnA.input(gamepad.a);

        dpadUp.input(gamepad.dpad_up);
        dpadDown.input(gamepad.dpad_down);

        if(btnY.onPress()) {
            if(!isBusy) {
                isBusy = true;
                ThreadUtils.getExecutorService().submit(() -> {
                    try {
                        if(mode == Mode.Unlatch) {
                            latch();
                            ThreadUtils.sleep(500);
                            runToPosition(0);
                        } else {
                            unlatch();
                            runToPosition(770);
                        }

                        mode = (mode == Mode.Latch) ? Mode.Unlatch : Mode.Latch;
                    } finally {
                        isBusy = false;
                    }
                });
            }
        } else if(btnA.onPress()) {
            if(!isBusy) {
                isBusy = true;
                ThreadUtils.getExecutorService().submit(() -> {
                    try {
                        runToPosition(450);
                        unlatch();
                        mode = Mode.Unlatch;
                    } finally {
                        isBusy = false;
                    }
                });
            }
        } else if(!isBusy) {
            if (gamepad.dpad_up) {
                wobbleMotor.setPower(0.1);
            } else if (gamepad.dpad_down) {
                wobbleMotor.setPower(-0.1);
            } else {
                wobbleMotor.setPower(0.0);
            }
        }

        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("wobbleMotor", wobbleMotor.getCurrentPosition());
    }

    public void latch() {
        wobbleServo.setPosition(SERVO_CLOSE);
    }

    public void unlatch() {
        wobbleServo.setPosition(SERVO_OPEN);
    }


    private void runToPosition(int position) {
        wobbleMotor.setTargetPosition(position);
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, wobbleMotor);
        wobbleMotor.setPower(0.3);

        while(OpModeUtils.opModeIsActive() && MotorUtils.motorIsBusy(wobbleMotor)) {
            // wait for motor to reach target position
        }

        wobbleMotor.setPower(0d);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, wobbleMotor);
    }
}
