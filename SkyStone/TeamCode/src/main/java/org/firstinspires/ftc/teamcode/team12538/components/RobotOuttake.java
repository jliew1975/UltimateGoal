package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.team12538.robot.CommonRobotHardware;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.states.Button;

public class RobotOuttake implements RobotComponent, ControlAware, TelemetryAware {
    public RobotStoneClaw outtakeClaw = new RobotStoneClaw();
    public RobotOuttakeSlides outtakeSlides = new RobotOuttakeSlides();
    public RobotStoneAligner aligner = new RobotStoneAligner();

    private Object outtakeLock = new Object();

    private volatile ClawMode clawMode = ClawMode.Open;

    private volatile boolean busy = false;

    public volatile int stoneHeight = 1;

    Button buttonA = new Button();
    Button buttonB = new Button();
    Button rightTrigger = new Button();

    @Override
    public void init() {
        outtakeClaw.init();
        outtakeSlides.init();
        aligner.init();
    }

    @Override
    public void control(Gamepad gamepad) {

        // outtake claw controls
        rightTrigger.input(gamepad.right_trigger > 0);
        if(rightTrigger.onPress()) {
            if(!busy) {
                synchronized (outtakeLock) {
                    if(!busy) {
                        busy = true;
                        performOuttakeClawOperation();
                    }
                }
            }
        }

        // stone pickup operation
        buttonA.input(gamepad.a);
        if(buttonA.onPress()) {
            if(!busy) {
                synchronized (outtakeLock) {
                    if(!busy) {
                        busy = true;
                        ThreadUtils.getExecutorService().submit(new Runnable() {
                            @Override
                            public void run() {
                                try {
                                    performStoneIntakeOperation();
                                } finally {
                                    busy = false;
                                }
                            }
                        });
                    }
                }
            }
        }

        // stone deployment operation
        buttonB.input(gamepad.b);
        if(buttonB.onPress()) {
            if(!busy) {
                synchronized (outtakeLock) {
                    if(!busy) {
                        busy = true;
                        ThreadUtils.getExecutorService().submit(() -> {
                            try {
                                prepareForStoneDeployment();
                            } finally {
                                busy = false;
                            }
                        });
                    }
                }
            }
        }

        if(!busy) {
            if (gamepad.dpad_up) {
                outtakeSlides.setPower(1d);
            } else if (gamepad.dpad_down) {
                outtakeSlides.setPower(-1d);
            } else {
                if(OpModeUtils.getGlobalStore().isLiftOuttake()) {
                    synchronized (outtakeLock) {
                        if(!busy) {
                            busy = true;
                            ThreadUtils.getExecutorService().submit(() -> {
                                try {
                                    liftSlideForStoneIntake();
                                } finally {
                                    busy = false;
                                }
                            });
                        }
                    }
                } else {
                    outtakeSlides.setPower(0);
                }
            }
        }
    }

    @Override
    public void printTelemetry() {
        outtakeClaw.printTelemetry();
        outtakeSlides.printTelemetry();
    }

    public void performStonePickupOperation() {
        OpModeUtils.getGlobalStore().setDepositMode(false);

        if (outtakeSlides.getCurrentPosition() < 250 && outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_DEPLOYMENT_POSITION) {
            ThreadUtils.getExecutorService().submit(() -> liftSlide());
        }

        while(OpModeUtils.opModeIsActive() && outtakeSlides.getCurrentPosition() < 1200) {
            ThreadUtils.idle();
        }

        // make sure arm and claw position is correct
        outtakeClaw.setArmPosition(RobotStoneClaw.ARM_STONE_INTAKE_POSITION);
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);

        lowerSlideForStonePickup();
    }

    public void performStoneIntakeOperation() {
        OpModeUtils.getGlobalStore().setDepositMode(false);

        if (outtakeSlides.getCurrentPosition() < RobotOuttakeSlides.ENCODER_TICKS_FOR_DEPLOY &&
                outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_DEPLOYMENT_POSITION)
        {
            ThreadUtils.getExecutorService().submit(() -> liftSlide());

            // wait for slide to go to safe height to move arm in
            while(OpModeUtils.opModeIsActive() && outtakeSlides.getCurrentPosition() < 1200) {
                ThreadUtils.idle();
            }
        }

        // make sure arm and claw position is correct
        outtakeClaw.setArmPosition(RobotStoneClaw.ARM_STONE_INTAKE_POSITION);
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);

        ThreadUtils.sleep(500);

        liftSlideForStoneIntake();
    }

    public void prepareForStoneDeployment() {
        OpModeUtils.getGlobalStore().setDepositMode(true);

        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_STONE_PICKUP);

        if(OpModeUtils.opModeIsActive()) {
            outtakeClaw.setArmPosition(RobotStoneClaw.ARM_STONE_PICKUP_POSITION);
            outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
            clawMode = ClawMode.Close;
        }

        ThreadUtils.sleep(200);

        // raise outtake slide
        if(OpModeUtils.getGlobalStore().isCap()) {
            outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_DEPLOY + 50);
        } else {
            ThreadUtils.getExecutorService().submit(() -> liftSlide());
        }

        while(OpModeUtils.opModeIsActive() && outtakeSlides.getCurrentPosition() < 1200) {
            ThreadUtils.idle();
        }

        if(OpModeUtils.opModeIsActive()) {
            // swing outtake arm out for stone deployment
            outtakeClaw.setArmPosition(RobotStoneClaw.ARM_DEPLOYMENT_POSITION);
            ThreadUtils.sleep(500);
        }

        if(OpModeUtils.opModeIsActive()) {
            outtakeSlides.runToStoneHeight(stoneHeight);
        }
    }

    public void liftSlideForStoneIntake() {
        clawMode = ClawMode.Open;
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);
        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_INTAKE);
    }

    public void lowerSlideForStonePickup() {
        RobotColorProximitySensor intakeSensor =
                OpModeUtils.getGlobalStore().getComponent(CommonRobotHardware.HardwareName.intakeSensor.name());

        if(intakeSensor.hasStoneButNotCompletelyIn()) {
            aligner.setPosition(RobotStoneAligner.ALIGN);
        }

        if(outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_STONE_INTAKE_POSITION) {
            outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);
        }

        lowerSlide();

        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_HOLD_POSITION);
        aligner.setPosition(RobotStoneAligner.INTAKE);
        ThreadUtils.sleep(300);

        // clam the stone for pickup
        clawMode = ClawMode.Close;
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
    }

    public void performOuttakeClawOperation() {
        ThreadUtils.getExecutorService().submit(() -> {
            try {
                if(clawMode == ClawMode.Open) {
                    if(outtakeClaw.getArmPosition() > RobotStoneClaw.ARM_DEPLOYMENT_POSITION) {
                        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_STONE_PICKUP);
                    }
                    outtakeClaw.setArmPosition(RobotStoneClaw.ARM_STONE_PICKUP_POSITION);
                    outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
                } else {
                    if (outtakeClaw.getArmPosition() > RobotStoneClaw.ARM_DEPLOYMENT_POSITION) {
                        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);
                    } else {
                        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                    }

                    ThreadUtils.sleep(250);

                    if (outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_DEPLOYMENT_POSITION &&
                            outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_DEPLOYMENT_POSITION) {
                        outtakeSlides.runToStoneHeight(stoneHeight);
                    }
                }
            } finally {
                clawMode =
                        (clawMode == ClawMode.Open) ?
                                ClawMode.Close : ClawMode.Open;
                busy = false;
            }
        });
    }

    private void liftSlide() {
        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_DEPLOY);
    }

    private void lowerSlide() {
        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_STONE_PICKUP, true);
    }
}
