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

import static org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils.sleep;

@Data
public class MineralMechanism implements RobotMechanic {
    public enum Direction { InTake, OutTake }
    public enum ArmDirection { Up, Down }
    public enum MineralSide { Left, Right, Both }

    private Object lock = new Object();

    private CRServo intake = null;
    private double intakeSpeed = 0.7;

    private Servo leftFlip = null;
    private Servo rightFlip = null;

    private DcMotor armExtension = null;

    private Servo depo = null;
    private DcMotor depoLift = null;

    private int upperLimit;
    private int lowerLimit;

    private int lowerSlowdownThreshold = 500;
    private int upperSlowdownThreshold = 9500;

    private double slowSpeed = 0.3;

    private volatile boolean busy = false;
    private volatile boolean swingArmBusy = false;
    private volatile boolean disableArmControl = false;

    private double depoLowerPos = 0.195;

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

        // intake continuous servo initialization
        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // intake box flipper servo initialization
        leftFlip = hardwareMap.servo.get("l_flip");
        rightFlip = hardwareMap.servo.get("r_flip");

        rightFlip.setDirection(Servo.Direction.REVERSE);

        if(!OpModeUtils.isDisableInitPos()) {
            leftFlip.setPosition(0.2);
            rightFlip.setPosition(0.2);
        }

        // mineral linear slide motor initialization
        armExtension = hardwareMap.get(DcMotor.class, "extend");
        armExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        if(OpModeUtils.getGlobalStore().isResetArmExtensionEncoderValue()) {
            MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, armExtension);
        }

        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, armExtension);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, armExtension);

        // deposit lift motor initialization
        depoLift = hardwareMap.get(DcMotor.class, "depo_lift");
        depoLift.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, depoLift);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, depoLift);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, depoLift);

        // Deposit box servo initialization
        depo = hardwareMap.get(Servo.class, "depo");
        depo.setDirection(Servo.Direction.REVERSE);
        depo.setPosition((depoLowerPos));
    }

    public void enableIntake(Direction direction) {
        if (direction == Direction.InTake) {
            intake.setPower(intakeSpeed);
        } else {
            intake.setPower(-intakeSpeed);
        }
    }

    public double getCollectorBoxPosition() {
        if(busy) {
            // when collector is performing autoMineralDeposit routine
            // return 0
            return 0d;
        }

        return Math.max(leftFlip.getPosition(), rightFlip.getPosition());
    }

    public void disableIntake() {
        intake.setPower(0);
    }

    public void flipCollectorBox(double position) {
        leftFlip.setPosition(position);
        rightFlip.setPosition(position);
    }

    public void jerkCollectorBox() {
        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                flipCollectorBox(0.3);
                sleep(30);
                flipCollectorBox(0.2);
            }
        });
    }

    public void controlArmExt(double power) {
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

    public void adjustArmExtPosition(final int positionDelta, final boolean isNewZeroPosInd) {
        synchronized (lock) {
            if (!busy) {
                busy = true;
                ThreadUtils.getExecutorService().submit(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            int targetPos = armExtension.getCurrentPosition() + positionDelta;
                            positionArmExt(targetPos);

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

    public void positionArmExt(int targetPosition) {
        positionArmExt(targetPosition, 1.0);
    }

    public void positionArmExt(int targetPosition, double power) {
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
                            // disableIntake();
                            flipCollectorBox(0.6);
                            positionArmExt(150, 1d);
                            flipCollectorBox(0.2d);
                            sleep(500);
                            // flipCollectorBox(0.4);
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

    public void liftDepo(int targetPosition, boolean isRotateDepoBox) {
        // make sure collector box is not in the way
        flipCollectorBox(0.6);
        sleep(500);

        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, depoLift);
        depoLift.setTargetPosition(targetPosition);
        depoLift.setPower(1);

        while(OpModeUtils.opModeIsActive() &&  depoLift.isBusy()) {
            // wait for motor to stop
        }

        // slowly rotate the depo servo
        if(isRotateDepoBox) {
            rotateDepositBox(0.4, 0.4);
        }
    }

    public void rotateDepositBox(final double slowPosition, final double finalPosition) {
        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                double depoInc = 0.05;
                double depoPosAdj = 0.2 - depoLowerPos;

                while(OpModeUtils.opModeIsActive() && depo.getPosition() < slowPosition) {
                    if(depo.getPosition() == depoLowerPos) {
                        depo.setPosition((depo.getPosition() + depoInc + depoPosAdj));
                    } else {
                        depo.setPosition(depo.getPosition() + depoInc);
                    }

                    sleep(100);
                }

                depo.setPosition(finalPosition);
                return;
            }
        });
    }

    public void jerkDepositBox() {
        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                depo.setPosition(0.9);
                sleep(30);
                depo.setPosition(1d);
            }
        });
    }

    public void lowerDepo() {
        depo.setPosition(0.4);
        depo.setPosition(depoLowerPos);

        flipCollectorBox(0.6);

        MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, depoLift);
        depoLift.setTargetPosition(10);
        depoLift.setPower(-1);

        while(OpModeUtils.opModeIsActive() && depoLift.isBusy()) {
            // intentionally left blank
        }

        depoLift.setPower(0);
    }

    public boolean canFlipDepoBox() {
        return depoLift.getCurrentPosition() > 1500;
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("armExtension", armExtension.getCurrentPosition());
        telemetry.addData("armExtensionPower", armExtension.getPower());
    }
}
