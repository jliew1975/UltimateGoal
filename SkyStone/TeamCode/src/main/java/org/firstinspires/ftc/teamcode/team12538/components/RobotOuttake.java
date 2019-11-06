package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.Gamepad;

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
            outtakeSlides.setPower(0.5);
        } else if(gamepad.dpad_down && !busy) {
            outtakeSlides.setPower(-0.5);
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
                                        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
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

    private void performStonePickupOperation() {
        if (outtakeSlides.getCurrentPosition() < 1750 && outtakeClaw.getArmPosition() == RobotStoneClaw.ARM_DEPLOYMENT_POSITION) {
            liftSlide();
        }

        ThreadUtils.sleep(200);

        // make sure arm and claw position is correct
        outtakeClaw.setArmPosition(RobotStoneClaw.ARM_STONE_PICKUP_POSITION);
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);

        // Wait for servo to complete its rotation before lowering down the slides
        ThreadUtils.sleep(500);

        lowerSlideForStonePickup();

        // clam the stone for pickup
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
        clawMode = ClawMode.Close;


    }

    private void prepareForStoneDeployment() {
        // make sure the claw is closed/clammed position
        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
        clawMode = ClawMode.Close;

        ThreadUtils.sleep(100);

        // raise outtake slide
        liftSlideForStoneIntake();

        // swing outtake arm out for stone deployment
        outtakeClaw.setArmPosition(RobotStoneClaw.ARM_DEPLOYMENT_POSITION);
    }

    private void liftSlideForStoneIntake() {
        liftSlide();
    }

    private void lowerSlideForStonePickup() {
        lowerSlide();
    }

    private void liftSlide() {
        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_INTAKE);
    }

    private void lowerSlide() {
        outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_STONE_PICKUP);
    }

    private void performOuttakeClawOperation() {
        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                try {
                    if(clawMode == ClawMode.Open) {
                        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
                    } else {
                        outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
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
}
