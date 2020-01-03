package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.team12538.robot.Robot;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class RobotOuttake implements RobotComponent, ControlAware, TelemetryAware {
    public RobotStoneClaw outtakeClaw = new RobotStoneClaw();
    public RobotOuttakeSlides outtakeSlides = new RobotOuttakeSlides();

    private Object outtakeLock = new Object();

    private volatile ClawMode clawMode = ClawMode.Open;

    private volatile boolean busy = false;

    public volatile int stoneHeight = 1;

    @Override
    public void init() {
        outtakeClaw.init();
        outtakeSlides.init();
    }

    @Override
    public void control(Gamepad gamepad) {
        // lift slides for intake
        if(gamepad.y) {
            if(!busy) {
                synchronized (outtakeLock) {
                    if(!busy) {
                        busy = true;
                        OpModeUtils.getGlobalStore().setLiftOuttake(false);

                        ThreadUtils.getExecutorService().submit(new Runnable() {
                            @Override
                            public void run() {
                                try {
                                    liftSlideForStoneIntake();
                                } finally {
                                    busy = false;
                                }
                            }
                        });
                    }
                }
            }
        }

        // outtake claw controls
        if(gamepad.right_trigger > 0) {
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
        if(gamepad.a) {
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
        if(gamepad.b) {
            if(!busy) {
                synchronized (outtakeLock) {
                    if(!busy) {
                        busy = true;
                        ThreadUtils.getExecutorService().submit(new Runnable() {
                            @Override
                            public void run() {
                                try {
                                    prepareForStoneDeployment();
                                } finally {
                                    busy = false;
                                }
                            }
                        });
                    }
                }
            }
        }

        // outtake slides controls
        if(gamepad.dpad_up && !busy) {
            outtakeSlides.runToPosition(outtakeSlides.getCurrentPosition() + 100);
        } else if(gamepad.dpad_down && !busy) {
            outtakeSlides.runToPosition(outtakeSlides.getCurrentPosition() - 100);
        } else {
            if(OpModeUtils.getGlobalStore().isLiftOuttake()) {
                if(!busy) {
                    synchronized (outtakeLock) {
                        if(!busy) {
                            busy = true;
                            OpModeUtils.getGlobalStore().setLiftOuttake(false);
                            ThreadUtils.getExecutorService().submit(() -> {
                                try {
                                    liftSlideForStoneIntake();
                                } finally {
                                    busy = false;
                                }
                            });
                        }
                    }
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

        // make sure arm and claw position is correct
        outtakeClaw.setArmPosition(RobotStoneClaw.ARM_STONE_PICKUP_POSITION);
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);

        // Wait for servo to complete its rotation before lowering down the slides
        ThreadUtils.sleep(500);

        lowerSlideForStonePickup();
    }

    public void performStoneIntakeOperation() {
        OpModeUtils.getGlobalStore().setDepositMode(false);

        if (outtakeSlides.getCurrentPosition() < RobotOuttakeSlides.ENCODER_TICKS_FOR_DEPLOY &&
                outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_DEPLOYMENT_POSITION)
        {
            ThreadUtils.getExecutorService().submit(() -> liftSlide());
        }

        // make sure arm and claw position is correct
        outtakeClaw.setArmPosition(RobotStoneClaw.ARM_STONE_PICKUP_POSITION);
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);

        // Wait for servo to complete its rotation before lowering down the slides
        ThreadUtils.sleep(500);

        liftSlideForStoneIntake();
    }

    public void prepareForStoneDeployment() {
        OpModeUtils.getGlobalStore().setDepositMode(true);

        ThreadUtils.getExecutorService().submit(() -> outtakeSlides.runToPosition(0, true));
        ThreadUtils.sleep(100);

        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
        clawMode = ClawMode.Close;
        ThreadUtils.sleep(500);

        // raise outtake slide
        ThreadUtils.getExecutorService().submit(() -> liftSlide());
        ThreadUtils.sleep(300);

        // swing outtake arm out for stone deployment
        outtakeClaw.setArmPosition(RobotStoneClaw.ARM_DEPLOYMENT_POSITION);
        ThreadUtils.sleep(500);

        outtakeSlides.runToStoneHeight(stoneHeight);
    }

    public void liftSlideForStoneIntake() {
        clawMode = ClawMode.Open;
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);
        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_INTAKE, true);
    }

    public void lowerSlideForStonePickup() {
        if(outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_STONE_PICKUP_POSITION) {
            outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);
        }

        lowerSlide();

        // clam the stone for pickup
        clawMode = ClawMode.Close;
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
    }

    public void performOuttakeClawOperation() {
        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                try {
                    if(clawMode == ClawMode.Open) {
                        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_STONE_PICKUP);
                        ThreadUtils.sleep(500);
                        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
                    } else {
                        if (outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_DEPLOYMENT_POSITION) {
                            outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                        } else {
                            outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);
                        }
                    }
                    ThreadUtils.sleep(500);

                    if(outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_DEPLOYMENT_POSITION) {
                        ThreadUtils.getExecutorService().submit(() -> outtakeSlides.runToStoneHeight(stoneHeight));
                    }
                } finally {
                    clawMode =
                            (clawMode == ClawMode.Open) ?
                                    ClawMode.Close : ClawMode.Open;
                    busy = false;
                }
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
