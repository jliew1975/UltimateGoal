package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public enum FlipPosition { Down, Up, Prepare }

    private Object lock = new Object();

    private CRServo intake = null;
    private double intakeSpeed = 0.7;

    // private Servo leftFlip = null;
    // private Servo rightFlip = null;
    private Servo intakeFlip = null;

    private DcMotor armExtension = null;

    private Servo depo = null;
    private DcMotor depoLift = null;

    private DigitalChannel magneticLimitSensor = null;

    private int upperLimit;
    private int lowerLimit;

    private int lowerSlowdownThreshold = 300;
    private int upperSlowdownThreshold = 9500;

    private double slowSpeed = 0.3;

    private volatile boolean busy = false;
    private volatile boolean intakeFlipIsBusy = false;
    private volatile boolean swingArmBusy = false;
    private volatile boolean disableArmControl = false;
    private volatile boolean intakeIsNotBusy = true;

    private volatile boolean depoBoxBusy = false;
    private volatile boolean depoLiftBusy = false;

    public double intakeFlipUpPos = 1.00;
    public double intakeFlipDownPos = 0.25;
    public double intakeFlipPrepPos = 0.50;

    private double depoLowerPos = 0.40;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        // intake continuous servo initialization
        intake = hardwareMap.get(CRServo.class, "intake");

        // intake box flipper servo initialization
        // leftFlip = hardwareMap.servo.get("l_flip");
        // rightFlip = hardwareMap.servo.get("r_flip");

        // intakeFlip initialization logic
        intakeFlip = hardwareMap.get(Servo.class, "intake_flip");
        intakeFlip.setPosition(intakeFlipUpPos);

        if(OpModeUtils.getGlobalStore().isResetArmExtensionEncoderValue()) {
            // MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, intakeFlip);
        }

        // mineral linear slide motor initialization
        armExtension = hardwareMap.get(DcMotor.class, "linear_slides");
        armExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        if(OpModeUtils.getGlobalStore().isResetArmExtensionEncoderValue()) {
            MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, armExtension);
        }

        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, armExtension);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, armExtension);

        // deposit lift motor initialization
        depoLift = hardwareMap.get(DcMotor.class, "vertical_slides");
        depoLift.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, depoLift);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, depoLift);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, depoLift);

        magneticLimitSensor = hardwareMap.get(DigitalChannel.class, "limit_switch");

        // Deposit box servo initialization
        depo = hardwareMap.get(Servo.class, "depo");
        depo.setDirection(Servo.Direction.REVERSE);
        depo.setPosition((depoLowerPos));
    }

    public void enableIntake(Direction direction) {
        intakeIsNotBusy = false;

        if (direction == Direction.InTake) {
            intake.setPower(intakeSpeed);
        } else {
            intake.setPower(-intakeSpeed);
        }
    }

    public void disableIntake() {
        intake.setPower(0);
        intakeIsNotBusy = true;
    }

    public boolean intakeIsNotBusy() {
        return intakeIsNotBusy;
    }

    public void flipCollectorBox() {
        synchronized (intakeFlip) {
            if (!intakeFlipIsBusy) {
                intakeFlipIsBusy = true;
                ThreadUtils.getExecutorService().submit(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            double intakePos = 0d;
                            if (intakeFlip.getPosition() == intakeFlipUpPos) {
                                intakePos = intakeFlipDownPos;
                            } else {
                                intakePos = intakeFlipUpPos;
                            }

                            intakeFlip.setPosition(intakePos);
                            sleep(500);
                        } finally {
                            intakeFlipIsBusy = false;
                        }
                    }
                });
            }
        }
    }

    public void flipCollectorBox(double position) {
        intakeFlip.setPosition(position);
    }

    public void controlArmExt(double power) {
        if(armExtension != null) {
            synchronized (armExtension) {
                if (disableArmControl) {
                    return;
                }
            }

            // power value: negative => retract, positive => extend
            double effectivePower = power;

            boolean slowDownArmMovement = false;
            boolean disableArmExtMovement = false;

            if(power > 0d && armExtension.getCurrentPosition() > upperSlowdownThreshold) {
                slowDownArmMovement = true;
            } else if(power < 0d && armExtension.getCurrentPosition() < lowerSlowdownThreshold) {
                slowDownArmMovement = true;
            }

            if(power < 0d && magneticLimitSensor.getState() == false) {
                disableArmExtMovement = true;
            } else if(power > 0d && armExtension.getCurrentPosition() > upperLimit) {
                // disableArmExtMovement = true;
            }

            if(slowDownArmMovement) {
                effectivePower = Math.signum(effectivePower) * 0.2;
            }

            if(disableArmExtMovement) {
                effectivePower = 0d;
            }

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

                }

                armExtension.setPower(0);
            } finally {
                disableArmControl = false;
                MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, armExtension);
            }
        }
    }

    public void positionArmExtForMineralTransfer() {
        if(armExtension != null) {
            disableArmControl = true;

            try {
                double effectivePower = 1d;
                int currentPosition = armExtension.getCurrentPosition();

                if (0 < currentPosition) {
                    effectivePower = -1 * effectivePower;
                }

                while (OpModeUtils.opModeIsActive() && magneticLimitSensor.getState()) {
                    currentPosition = armExtension.getCurrentPosition();
                    if(currentPosition < lowerSlowdownThreshold) {
                        effectivePower = Math.signum(effectivePower) * 0.2;
                    }

                    armExtension.setPower(effectivePower);
                }

                armExtension.setPower(0);
            } finally {
                disableArmControl = false;
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
                            flipCollectorBox(intakeFlipPrepPos);
                            positionArmExtForMineralTransfer();
                            flipCollectorBox(intakeFlipUpPos);
                            sleep(500);
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

    public void liftDepo(final int targetPosition) {
        synchronized (depoLift) {
            if(!depoLiftBusy) {
                depoLiftBusy = true;

                ThreadUtils.getExecutorService().submit(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, depoLift);
                            depoLift.setTargetPosition(targetPosition);
                            depoLift.setPower(1);

                            while(OpModeUtils.opModeIsActive() &&  depoLift.isBusy()) {
                                // wait for motor to stop
                            }
                        } finally {
                            depoLiftBusy = false;
                        }
                    }
                });
            }
        }
    }

    public void lowerDepo() {
        synchronized (depoLift) {
            if(!depoLiftBusy) {
                depoLiftBusy = true;

                ThreadUtils.getExecutorService().submit(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            depo.setPosition(depoLowerPos);

                            MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, depoLift);
                            depoLift.setTargetPosition(0);
                            depoLift.setPower(-1);

                            while (OpModeUtils.opModeIsActive() && depoLift.isBusy()) {
                                // intentionally left blank
                            }

                            depoLift.setPower(0);
                        } finally {
                            depoLiftBusy = false;
                        }
                    }
                });
            }
        }
    }

    public void rotateDepositBox(final double finalPosition) {
        synchronized (depo) {
            if(!depoBoxBusy) {
                depoBoxBusy = true;

                ThreadUtils.getExecutorService().submit(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            if(depo.getPosition() == finalPosition) {
                                jerkDepoBox();
                            } else {
                                depo.setPosition(finalPosition);
                            }
                        } finally {
                            depoBoxBusy = false;
                        }
                    }
                });
            }
        }
    }

    public void jerkDepoBox() {
        depo.setPosition(0.8);
        sleep(100);
        depo.setPosition(1d);
    }

    public boolean canFlipDepoBox() {
        return depoLift.getCurrentPosition() > 1500;
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("intakeFlip", intakeFlip.getPosition());
        telemetry.addData("armExtension", armExtension.getCurrentPosition());
        telemetry.addData("armExtensionPower", armExtension.getPower());
        telemetry.addData("limitSwitch", magneticLimitSensor.getState());
    }
}
