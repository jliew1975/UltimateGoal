package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.team12538.robot.Robot;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class RobotOuttake implements RobotComponent, ControlAware, TelemetryAware {
    public RobotStoneClaw outtakeClaw = new RobotStoneClaw();
    public RobotOuttakeSlides outtakeSlides = new RobotOuttakeSlides();

    private Object outtakeLock = new Object();

    private volatile ClawMode clawMode = ClawMode.Open;

    private volatile boolean busy = false;
    private volatile boolean inReadyState = false;

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
                                    performStonePickupOperation();
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
            outtakeSlides.setPower(1d);
        } else if(gamepad.dpad_down && !busy) {
            outtakeSlides.setPower(-1d);
        } else {
            if(OpModeUtils.getGlobalStore().isLiftOuttake()) {
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
            } else if(!busy) {
                outtakeSlides.setPower(0d);
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

        if(outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_DEPLOYMENT_POSITION) {
            RobotFoundationClaw foundationClaw = OpModeUtils.getGlobalStore().getComponent("foundationClaw");
            foundationClaw.setClawPosition(RobotFoundationClaw.A_POSITION);
        }

        if (outtakeSlides.getCurrentPosition() < 1750 && outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_DEPLOYMENT_POSITION) {
            liftSlide();
        }

        RobotFoundationClaw foundationClaw = OpModeUtils.getGlobalStore().getComponent("foundationClaw");
        foundationClaw.setClawPosition(RobotFoundationClaw.LOWER_CLAW_POS);

        ThreadUtils.sleep(200);

        // make sure arm and claw position is correct
        outtakeClaw.setArmPosition(RobotStoneClaw.ARM_STONE_PICKUP_POSITION);
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);

        // Wait for servo to complete its rotation before lowering down the slides
        ThreadUtils.sleep(500);

        lowerSlideForStonePickup();
    }

    public void prepareForStoneDeployment() {
        /*
        // make sure the claw is closed/clammed position
        clawMode = ClawMode.Close;
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
        */

        OpModeUtils.getGlobalStore().setDepositMode(true);

        ThreadUtils.sleep(100);

        // raise outtake slide
        liftSlide();

        // swing outtake arm out for stone deployment
        outtakeClaw.setArmPosition(RobotStoneClaw.ARM_DEPLOYMENT_POSITION);

        ThreadUtils.sleep(500);

        RobotFoundationClaw foundationClaw = OpModeUtils.getGlobalStore().getComponent("foundationClaw");
        foundationClaw.setClawPosition(RobotFoundationClaw.A_POSITION);
    }

    public void liftSlideForStoneIntake() {
        clawMode = ClawMode.Open;
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);
        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_INTAKE);
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
                        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
                    } else {
                        if (outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_DEPLOYMENT_POSITION) {
                            outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                        } else {
                            outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);
                        }
                    }
                    ThreadUtils.sleep(500);
                } finally {
                    clawMode =
                            (clawMode == ClawMode.Open) ?
                                    ClawMode.Close : ClawMode.Open;
                    busy = false;
                }
            }
        });
    }

    public void conttrolForCapstone(Gamepad gamepad) {
        if(gamepad.right_bumper) {
            outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_CAPSTONE_POSITION);
        }
    }

    private void liftSlide() {
        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_DEPLOY);
    }

    private void lowerSlide() {
        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_STONE_PICKUP);
    }
}
