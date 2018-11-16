package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class MineralMechanism implements RobotMechanic {
    public enum Direction { InTake, OutTake }
    public enum MineralSide { Left, Right, Both }

    private Object lock = new Object();

    private DcMotor intake = null;
    private double intakeSpeed = 0.7;

    private Servo leftArm = null;
    private Servo rightArm = null;
    private DcMotor armExtension = null;

    private Servo leftRelease = null;
    private Servo rightRelease = null;

    private DcMotor swingingArm = null;

    private int upperLimit;
    private int lowerLimit;

    private int lowerSlowdownThreshold = 500;
    private int upperSlowdownThreshold = 9500;

    private double slowSpeed = 0.3;

    private volatile boolean busy = false;
    private volatile boolean swingArmBusy = false;
    private volatile boolean disableArmControl = false;

    private volatile boolean intakeAutoOn = false;

    public MineralMechanism(int lowerLimit, int upperLimit) {
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;

        if(upperLimit < upperSlowdownThreshold) {
            upperSlowdownThreshold = upperLimit - 500;
        }

        if(lowerLimit > lowerSlowdownThreshold) {
            lowerSlowdownThreshold = lowerLimit + 500;
        }
    }

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, intake);

        leftArm = hardwareMap.servo.get("l_flip");
        rightArm = hardwareMap.servo.get("r_flip");
        rightArm.setDirection(Servo.Direction.REVERSE);

        leftArm.setPosition(0.9);
        rightArm.setPosition(0.9);

        armExtension = hardwareMap.get(DcMotor.class, "extend");
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, armExtension);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, armExtension);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, armExtension);

        swingingArm = hardwareMap.get(DcMotor.class, "swing");
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, swingingArm);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, swingingArm);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, swingingArm);

        leftRelease = hardwareMap.get(Servo.class, "l_depo");
        rightRelease = hardwareMap.get(Servo.class, "r_depo");
        rightRelease.setDirection(Servo.Direction.REVERSE);

        leftRelease.setPosition(0d);
        rightRelease.setPosition(0d);
    }

    public void enableIntake(Direction direction) {
        enableIntake(direction, false);
    }

    public void enableIntake(Direction direction, boolean intakeAutoOn) {
        if(direction == Direction.InTake) {
            intake.setPower(intakeSpeed);
            this.intakeAutoOn = intakeAutoOn;
        } else {
            intake.setPower(-intakeSpeed);
        }
    }

    public void disableIntake() {
        intake.setPower(0);
    }

    public boolean isIntakeAutoOn() {
        return intakeAutoOn;
    }

    public void flipCollectorBox(double position) {
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }

    public void controlArm(double power) {
        if(armExtension != null) {
            synchronized (armExtension) {
                if (disableArmControl) {
                    return;
                }
            }

            double effectivePower = power;

            int curPos = armExtension.getCurrentPosition();
            if (power > 0 && curPos < upperLimit ) {
                if(curPos >= upperSlowdownThreshold) {
                    effectivePower = Math.signum(power) * slowSpeed;
                }

                armExtension.setPower(effectivePower);
            } else if (power < 0 && curPos >= lowerLimit) {
                if(curPos <= lowerSlowdownThreshold) {
                    effectivePower = Math.signum(power) * slowSpeed;
                }

                armExtension.setPower(effectivePower);
            } else {
                armExtension.setPower(0d);
            }

            if(curPos < 0) {
                MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, armExtension);
            }
        }
    }

    public void postionArm(int targetPosition) {
        if(armExtension != null) {
            disableArmControl = true;

            try {
                MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, armExtension);
                armExtension.setTargetPosition(targetPosition);
                armExtension.setPower(1.0);
                while (OpModeUtils.opModeIsActive() && armExtension.isBusy()) {
                    ThreadUtils.idle();
                }
            } finally {
                disableArmControl = false;
                MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, armExtension);
            }
        }
    }

    public void autoMineralDeposit() {
        synchronized (lock) {
            if (!busy) {
                busy = true;

                ThreadUtils.getExecutorService().submit(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            RobotLog.d("starting autoMineralDeposit logic");
                            disableIntake();
                            flipCollectorBox(0.6);
                            postionArm(255);
                            flipCollectorBox(1d);
                            ThreadUtils.sleep(500);
                            flipCollectorBox(0.6);
                            RobotLog.d("done with autoMineralDeposit logic");
                        } finally {
                            busy = false;
                        }
                    }
                });
            }
        }
    }

    public void swingArm(double power) {
        swingingArm.setPower(power);
    }

    public void swingArmPositionBy(final int targetPosition) {
        synchronized (lock) {
            if(!swingArmBusy) {
                swingArmBusy = true;

                ThreadUtils.getExecutorService().submit(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            int calculatedTargetPosition = swingingArm.getCurrentPosition() + targetPosition;
                            swingArmToPosition(calculatedTargetPosition, 0.2);
                            while(OpModeUtils.opModeIsActive() && swingingArm.isBusy()) {
                                ThreadUtils.idle();
                            }
                        } finally {
                            swingArmBusy = false;
                        }
                    }
                });
            }
        }
    }

    public void swingArmToPosition(int targetPosition, double speed) {
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, swingingArm);
        swingingArm.setTargetPosition(targetPosition);
        swingingArm.setPower(speed);
    }

    public boolean isSwingArmUp() {
        return swingingArm.getCurrentPosition() > 150;
    }

    public void controlReleaseMineral(MineralSide side, double position) {
        if(side == MineralSide.Left) {
            leftRelease.setPosition(position);
        } else if(side == MineralSide.Right) {
            rightRelease.setPosition(position);
        } else {
            leftRelease.setPosition(position);
            rightRelease.setPosition(position);
        }
    }

    public int getLowerSlowdownThreshold() {
        return lowerSlowdownThreshold;
    }

    public void setLowerSlowdownThreshold(int lowerSlowdownThreshold) {
        this.lowerSlowdownThreshold = lowerSlowdownThreshold;
    }

    public int getUpperSlowdownThreshold() {
        return upperSlowdownThreshold;
    }

    public void setUpperSlowdownThreshold(int upperSlowdownThreshold) {
        this.upperSlowdownThreshold = upperSlowdownThreshold;
    }

    public double getSlowSpeed() {
        return slowSpeed;
    }

    public void setSlowSpeed(double slowSpeed) {
        this.slowSpeed = slowSpeed;
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("armExtension", armExtension.getCurrentPosition());
        telemetry.addData("disableArmControl", disableArmControl);
        telemetry.addData("swingingArm", swingingArm.getCurrentPosition());
    }
}
