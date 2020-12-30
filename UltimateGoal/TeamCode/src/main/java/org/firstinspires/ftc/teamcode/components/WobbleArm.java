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

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class WobbleArm implements RobotComponent, Runnable {
    public static final double SERVO_OPEN = 0d;
    public static final double SERVO_CLOSE = 0.8;

    public static final int WOBBLE_ARM_DOWN = 500;
    public static final int WOBBLE_ARM_PARTIAL_DOWN = 280;
    public static final int WOBBLE_ARM_LIFT = -200;

    private enum Mode {DownForPickup, LiftForPickup, TargetZoneDrop, DropZoneDrop, ManualAdjustUp, ManualAdjustDown, Idle}

    private Servo wobbleServo;
    private DcMotorEx wobbleMotor;

    ExecutorService executorService = Executors.newSingleThreadExecutor();

    private ElapsedTime runtime = new ElapsedTime();

    private volatile boolean isBusy = false;
    private Button btnX = new Button();
    private Button btnY = new Button();
    private Button btnA = new Button();
    private Button btnB = new Button();

    private Button dpadUp = new Button();
    private Button dpadDown = new Button();

    private volatile Mode mode = Mode.Idle;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        wobbleMotor = hardwareMap.get(DcMotorEx.class, "wobbleMotor");

        wobbleServo.setDirection(Servo.Direction.REVERSE);
        wobbleServo.setPosition(SERVO_CLOSE);

        if (OpModeUtils.getGlobalStore().getRunMode() == OpModeStore.RunMode.Autonomous) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            executorService.submit(this);
        }

        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setPositionPIDFCoefficients(2.0);

        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void control(Gamepad gamepad) {
        btnY.input(gamepad.x);
        btnY.input(gamepad.y);
        btnA.input(gamepad.a);
        btnB.input(gamepad.b);

        dpadUp.input(gamepad.dpad_up);
        dpadDown.input(gamepad.dpad_down);

        if (btnX.onPress()) {
            mode = Mode.TargetZoneDrop;
        } else if (btnY.onPress()) {
            mode = Mode.LiftForPickup;
        } else if (btnA.onPress()) {
            mode = Mode.DownForPickup;
        } else if (btnB.onPress()) {
            mode = Mode.DropZoneDrop;
        } else if (dpadUp.isPressed()) {
            mode = Mode.ManualAdjustUp;
        } else if (dpadDown.isPressed()) {
            mode = Mode.ManualAdjustDown;
        }

        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("wobbleMode", mode);
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
        while (OpModeUtils.opModeIsActive() && MotorUtils.motorIsBusy(wobbleMotor)) {
            // wait for motor to reach target position
            if (runtime.milliseconds() > 1000) {
                break;
            }
        }

        wobbleMotor.setPower(0d);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, wobbleMotor);
    }

    public void run() {
        while (OpModeUtils.opModeIsActive()) {
            try {
                switch (mode) {
                    case DownForPickup:
                        prepareWobbleArmForPickup();
                        mode = Mode.Idle;
                        break;
                    case LiftForPickup:
                        liftUpWobble();
                        mode = Mode.Idle;
                        break;
                    case TargetZoneDrop:
                        fullWobbleDrop();
                        mode = Mode.Idle;
                        break;
                    case DropZoneDrop:
                        partialWobbleDrop();
                        mode = Mode.Idle;
                        break;
                    case ManualAdjustUp:
                        manualAdjustUp();
                        mode = Mode.Idle;
                        break;
                    case ManualAdjustDown:
                        manualAdjustDown();
                        mode = Mode.Idle;
                        break;
                    case Idle:
                        wobbleMotor.setPower(0d);
                }
            } catch (Throwable th) {
                th.printStackTrace();
                mode = Mode.Idle;
            }
        }

        executorService.shutdownNow();
    }

    private void prepareWobbleArmForPickup() {
        wobbleServo.setPosition(SERVO_OPEN);
        runToPosition(WOBBLE_ARM_DOWN);
    }

    private void liftUpWobble() {
        wobbleServo.setPosition(SERVO_CLOSE);
        ThreadUtils.sleep(500); // wait for servo to reach target position
        runToPosition(WOBBLE_ARM_LIFT);
    }

    private void fullWobbleDrop() {
        runToPosition(WOBBLE_ARM_DOWN);
        wobbleServo.setPosition(SERVO_OPEN);
    }

    private void partialWobbleDrop() {
        runToPosition(WOBBLE_ARM_PARTIAL_DOWN);
        if(wobbleMotor.getCurrentPosition() > 250) {
            wobbleServo.setPosition(SERVO_OPEN);
        }
    }

    private void manualAdjustUp() {
        if (wobbleMotor.getCurrentPosition() > -200) {
            wobbleMotor.setPower(-0.5);
        } else {
            wobbleMotor.setPower(0d);
        }
    }

    private void manualAdjustDown() {
        if (wobbleMotor.getCurrentPosition() < 630) {
            wobbleMotor.setPower(0.5);
        } else {
            wobbleMotor.setPower(0d);
        }
    }
}
