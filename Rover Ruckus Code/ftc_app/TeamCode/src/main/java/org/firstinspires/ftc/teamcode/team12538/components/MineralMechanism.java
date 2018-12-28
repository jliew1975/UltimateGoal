package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

import lombok.Data;

@Data
public class MineralMechanism implements RobotMechanic {
    public enum Direction { InTake, OutTake }
    public enum ArmDirection { Up, Down }
    public enum MineralSide { Left, Right, Both }

    private Object lock = new Object();

    private CRServo intake = null;
    private double intakeSpeed = 0.7;

    private Servo leftArm = null;
    private Servo rightArm = null;

    private DcMotor armExtension = null;

    private Servo parkingRod = null;

    // private Servo leftRelease = null;
    // private Servo rightRelease = null;

    // private Servo outtakeSlide = null;

    // private DcMotor swingingArm = null;

    private DcMotor depoLift = null;

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

        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArm = hardwareMap.servo.get("l_flip");
        rightArm = hardwareMap.servo.get("r_flip");

        rightArm.setDirection(Servo.Direction.REVERSE);

        if(!OpModeUtils.isDisableInitPos()) {
            leftArm.setPosition(0.2);
            rightArm.setPosition(0.2);
        }

        armExtension = hardwareMap.get(DcMotor.class, "extend");
        if(OpModeUtils.getGlobalStore().isResetArmExtensionEncoderValue()) {
            MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, armExtension);
        }

        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, armExtension);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, armExtension);

        depoLift = hardwareMap.get(DcMotor.class, "depo_lift");
        depoLift.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, depoLift);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, depoLift);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, depoLift);

        parkingRod = hardwareMap.get(Servo.class, "parking_rod");
        parkingRod.setPosition(0d);

        /* Old Mechanism
        swingingArm = hardwareMap.get(DcMotor.class, "swing_arm");
        swingingArm.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, swingingArm);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, swingingArm);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, swingingArm);


        leftRelease = hardwareMap.get(Servo.class, "l_depo");
        rightRelease = hardwareMap.get(Servo.class, "r_depo");
        rightRelease.setDirection(Servo.Direction.REVERSE);

        if(!OpModeUtils.isDisableInitPos()) {
            if (OpModeUtils.getGlobalStore().isCloseDepoArm()) {
                leftRelease.setPosition(1d);
                rightRelease.setPosition(1d);
            } else {
                leftRelease.setPosition(0d);
                rightRelease.setPosition(0d);
            }
        }

        outtakeSlide = hardwareMap.get(Servo.class, "outtake_slide");
        if(!OpModeUtils.isDisableInitPos()) {
            outtakeSlide.setPosition(0.5);
        }
        */
    }

    public void enableIntake(Direction direction) {
        enableIntake(direction, false);
    }

    public void enableIntake(Direction direction, boolean intakeAutoOn) {
        if (direction == Direction.InTake) {
            intake.setPower(intakeSpeed);
            this.intakeAutoOn = intakeAutoOn;
        } else {
            intake.setPower(-intakeSpeed);
        }
    }

    public boolean isNotCompletelyLowered() {
        return (leftArm.getPosition() < 1d || rightArm.getPosition() < 1d);
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

            // negative => extend, positive => retract
            double effectivePower = power;

            int curPos = armExtension.getCurrentPosition();
            armExtension.setPower(effectivePower);
        }
    }

    public void adjustArmPosition(final int positionDelta, final boolean isNewZeroPosInd) {
        synchronized (lock) {
            if (!busy) {
                busy = true;
                ThreadUtils.getExecutorService().submit(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            int targetPos = armExtension.getCurrentPosition() + positionDelta;
                            positionArm(targetPos);

                            if (isNewZeroPosInd) {
                                // reset the encoder and make that position the new zero position
                                MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, armExtension);
                                MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, armExtension);
                            }
                        } finally {
                            busy = false;
                        }
                    }
                });
            }
        }
    }

    public void positionArm(int targetPosition) {
        positionArm(targetPosition, 1.0);
    }

    public void positionArm(int targetPosition, double power) {
        if(armExtension != null) {
            disableArmControl = true;

            try {
                int currentPosition = armExtension.getCurrentPosition();
                double effectivePower = power;

                if(targetPosition < currentPosition) {
                    effectivePower = -1 * effectivePower;
                }

                MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, armExtension);
                armExtension.setTargetPosition(targetPosition);
                armExtension.setPower(effectivePower);

                while (OpModeUtils.opModeIsActive() && armExtension.isBusy()) {
                    ThreadUtils.idle();
                }

                armExtension.setPower(0);
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
                            positionArm(6000, 1d);
                            flipCollectorBox(1d);
                            ThreadUtils.sleep(500);
                            flipCollectorBox(0.4);
                            RobotLog.d("done with autoMineralDeposit logic");
                        } catch(Exception e) {
                            RobotLog.dd("MineralMechanism", e, e.getMessage());
                        } finally {
                            busy = false;
                        }
                    }
                });
            }
        }
    }

    /*
    public void swingArmSync(final ArmDirection armDirection) {
        synchronized (lock) {
            if (!swingArmBusy) {
                try {
                    swingArmBusy = true;
                    swingArm(armDirection);
                } finally {
                    swingArmBusy = false;
                }
            }
        }
    }

    public void swingArmAsync(final ArmDirection armDirection) {
        synchronized (lock) {
            if (!swingArmBusy) {
                swingArmBusy = true;

                ThreadUtils.getExecutorService().submit(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            swingArm(armDirection);
                        } finally {
                            swingArmBusy = false;
                        }
                    }
                });
            }
        }
    }

    private void swingArm(final ArmDirection armDirection) {
        int armCurrentPosition = swingingArm.getCurrentPosition();
        int targetPosition = armDirection == ArmDirection.Up ? 750 : 100;

        if(Math.abs(targetPosition - armCurrentPosition) < 10) {
            return;
        }

        if(armDirection == ArmDirection.Down) {
            swingArmToPosition(swingingArm.getCurrentPosition() - 100, 0.3);
            ThreadUtils.sleep(500);
        }

        if(armDirection == ArmDirection.Down) {
            outtakeSlide.setPosition(0.5);
            ThreadUtils.sleep(500);
        }

        swingArmToPosition(targetPosition, 0.3);

        ThreadUtils.sleep(500);
        outtakeSlide.setPosition(1d);

        if(armDirection == ArmDirection.Down) {
            swingArmToPosition(0, 0.3);
            swingingArm.setPower(0);
        }
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
                            swingArmToPosition(calculatedTargetPosition, 0.3);
                        } finally {
                            swingArmBusy = false;
                        }
                    }
                });
            }
        }
    }

    public void swingArmToPosition(int targetPosition, double power) {
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, swingingArm);
        int currentPosition = swingingArm.getCurrentPosition();

        double effectivePower = power;
        if(targetPosition < currentPosition) {
            effectivePower = -1 * effectivePower;
        }

        swingingArm.setTargetPosition(targetPosition);
        swingingArm.setPower(effectivePower);

        while(OpModeUtils.opModeIsActive() && swingingArm.isBusy()) {
            ThreadUtils.idle();
        }
    }

    public boolean isSwingReadyForRelease() {
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
    */

    public void liftDepo() {
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, depoLift);
        depoLift.setTargetPosition(3030);
        depoLift.setPower(1);
    }

    public void lowerDepo() {
        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, depoLift);
        depoLift.setTargetPosition(0);
        depoLift.setPower(-1);

        while(OpModeUtils.opModeIsActive() && depoLift.isBusy()) {

        }
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("armExtension", armExtension.getCurrentPosition());
        telemetry.addData("armExtensionPower", armExtension.getPower());
    }
}
