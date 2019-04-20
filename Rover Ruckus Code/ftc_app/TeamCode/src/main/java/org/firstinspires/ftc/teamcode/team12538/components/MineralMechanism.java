package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    private DcMotor intake = null;
    private double intakeSpeed = 1.0;

    private Servo intakeFlip = null;
    private Servo intakeGate = null;

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

    private volatile boolean depoBoxBusy = false;
    private volatile boolean depoLiftBusy = false;

    public double intakeFlipHangPos = 1.0;
    public double autoIntakeFlipUpPos = 1.0;
    public double intakeFlipUpPos = 1.0;
    public double intakeFlipDownPos = 0.2;
    public double intakeFlipPrepPos = 0.68;

    public double depoLowerPos = 0.05;
    public double depoFlipPos = 0.46;

    private double intakeGateOpen = 0d;
    private double intakeGateClose = 0.5;

    private int currentIntakeZeroPos = 0;

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime armTimeout = new ElapsedTime();
    private ElapsedTime armExtensionRuntime = new ElapsedTime();
    private ElapsedTime depoLiftRuntime = new ElapsedTime();

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        // intake motor initialization
        intake = hardwareMap.get(DcMotor.class, "intake");

        intakeGate = hardwareMap.get(Servo.class, "intake_gate");
        intakeGate.setPosition(intakeGateOpen);

        // intakeFlip initialization logic
        intakeFlip = hardwareMap.get(Servo.class, "intake_flip");
        intakeFlip.setPosition(intakeFlipUpPos);


        // mineral linear slide motor initialization
        armExtension = hardwareMap.get(DcMotor.class, "linear_slides");
        armExtension.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, armExtension);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, armExtension);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, armExtension);

        // deposit lift motor initialization
        depoLift = (DcMotor) hardwareMap.get(DcMotor.class, "vertical_slides");

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
        if (direction == Direction.InTake) {
            intake.setPower(-intakeSpeed);
        } else {
            intake.setPower(+intakeSpeed);
        }
    }

    public void disableIntake() {
        intake.setPower(0);
    }

    public void flipCollectorBox(double position) {
        if(position == intakeFlipUpPos && intakeFlip.getPosition() == position) {
            intakeFlip.setPosition(0.5);
            // sleep(300);
        }

        if(position == autoIntakeFlipUpPos) {
            intakeGate.setPosition(intakeGateOpen);
            sleep(400);
        }

        intakeFlip.setPosition(position);

        if(position != intakeFlipUpPos) {
            sleep(200);
            intakeGate.setPosition(intakeGateClose);
        }
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


            if(power < 0d && magneticLimitSensor.getState() == false) {
                disableArmExtMovement = true;
            }

            armExtension.setPower(effectivePower);
        }
    }

    public void positionArmExt(int targetPosition) {
        positionArmExt(targetPosition, 1.0, 5.0);
    }

    public void positionArmExt(int targetPosition, double power, double timeout) {
        if(armExtension != null) {
            disableArmControl = true;

            try {
                int calculatedTargetPos = armExtension.getCurrentPosition() + targetPosition;
                double effectivePower = power;

                MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, armExtension);
                armExtension.setTargetPosition(calculatedTargetPos);
                armExtension.setPower(effectivePower);

                armTimeout.reset();
                armExtensionRuntime.reset();

                int curPos = -1;
                while (OpModeUtils.opModeIsActive() && armExtension.isBusy()) {
                    if(curPos == -1 || armExtensionRuntime.seconds() >= 1) {
                        if(curPos != -1 && curPos == armExtension.getCurrentPosition()) {
                            break;
                        }

                        curPos = armExtension.getCurrentPosition();
                        armExtensionRuntime.reset();
                    }

                    if(armTimeout.seconds() > timeout) {
                        break;
                    }
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
                armExtensionRuntime.reset();

                int currPos = -1;
                while (OpModeUtils.opModeIsActive() && magneticLimitSensor.getState()) {
                    if(armExtension.getCurrentPosition() < (currentIntakeZeroPos + 300)) {
                        intakeGate.setPosition(intakeGateOpen);
                    }

                    if(armExtension.getCurrentPosition() < (currentIntakeZeroPos + 100)) {
                        armExtension.setPower(-0.3);
                    } else {
                        armExtension.setPower(-1.0);
                    }

                    if(currPos == -1 || armExtensionRuntime.seconds() > 0.5) {
                        if(currPos != -1 && currPos == armExtension.getCurrentPosition()) {
                            break;
                        }

                        currPos = armExtension.getCurrentPosition();
                        armExtensionRuntime.reset();
                    }
                }

                armExtension.setPower(0);
                currentIntakeZeroPos = armExtension.getCurrentPosition();
            } finally {
                disableArmControl = false;
            }
        }
    }

    public void autoMineralDeposit() {
        autoMineralDeposit(false);
    }

    public void autoMineralDeposit(boolean isAUto) {
        autoMineralDeposit(false, true);
    }

    public void autoMineralDeposit(final boolean isAuto, final boolean isTransfer) {
        if(isAuto) {
            autoMineralTransfer(isTransfer);
        } else {
            synchronized (lock) {
                if (!busy) {
                    busy = true;

                    ThreadUtils.getExecutorService().submit(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                mineralTransfer();
                            } catch (Exception e) {
                                RobotLog.dd("MineralMechanism", e, e.getMessage());
                            } finally {
                                busy = false;
                            }
                        }
                    });
                }
            }
        }
    }


    public void liftDepo(int targetPosition) {
        liftDepo(targetPosition, false);
    }

    public void liftDepo(final int targetPosition, boolean isAuto) {
        if(isAuto) {
            setLiftDepoPosition(targetPosition);
        } else {
            synchronized (depoLift) {
                if (!depoLiftBusy) {
                    depoLiftBusy = true;

                    ThreadUtils.getExecutorService().submit(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                setLiftDepoPosition(targetPosition);
                                rotateDepositBox(depoFlipPos);
                            } finally {
                                depoLiftBusy = false;
                            }
                        }
                    });
                }
            }
        }
    }

    public void lowerDepo() {
        lowerDepo(false);
    }

    public void lowerDepo(boolean isAuto) {
        if(isAuto) {
            setLiftDepoPosition(0);
        } else {
            synchronized (depoLift) {
                if (!depoLiftBusy) {
                    depoLiftBusy = true;

                    ThreadUtils.getExecutorService().submit(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                setLiftDepoPosition(0);
                            } finally {
                                depoLiftBusy = false;
                            }
                        }
                    });
                }
            }
        }
    }

    public void flipDepoBox() {
        rotateDepositBox(depoFlipPos, false);
    }

    public void flipDepoBox(boolean isAuto) {
        rotateDepositBox(depoFlipPos, isAuto);
    }

    public void rotateDepositBox(final double finalPosition) {
        rotateDepositBox(finalPosition, false);
    }

    public void rotateDepositBox(final double finalPosition, boolean isAuto) {
        if(isAuto) {
            rotateDepoBox(finalPosition);
            sleep(1000);
        } else {
            synchronized (depo) {
                if(!depoBoxBusy) {
                    depoBoxBusy = true;

                    ThreadUtils.getExecutorService().submit(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                rotateDepoBox(finalPosition);
                            } finally {
                                depoBoxBusy = false;
                            }
                        }
                    });
                }
            }
        }
    }

    public boolean canFlipDepoBox() {
        return depoLift.getCurrentPosition() > 200;
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("depoLift", depoLift.getCurrentPosition());
        telemetry.addData("armExtPosition", armExtension.getCurrentPosition());
        telemetry.addData("magneticLimitSensor", magneticLimitSensor.getState());
    }

    private void autoMineralTransfer() {
        autoMineralTransfer(true);
    }

    private void autoMineralTransfer(boolean isTransfer) {
        flipCollectorBox(intakeFlipPrepPos);

        positionArmExtForMineralTransfer();

        if(isTransfer) {
            flipCollectorBox(autoIntakeFlipUpPos);
        }
    }

    private void mineralTransfer() {
        RobotLog.d("starting autoMineralDeposit logic");

        flipCollectorBox(intakeFlipPrepPos);

        if(magneticLimitSensor.getState()) {
            positionArmExtForMineralTransfer();
        }

        // flipCollectorBox(intakeFlipUpPos);
        // sleep(100);
    }

    private void setLiftDepoPosition(int targetPosition) {
        setLiftDepoPosition(targetPosition, true);
    }

    private void setLiftDepoPosition(int targetPosition, boolean isFixStallScenario) {
        if(targetPosition == 0) {
            depo.setPosition(0);
            sleep(300);
            MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, depoLift);
            depoLift.setTargetPosition(targetPosition);
            depoLift.setPower(0.3);

            depoLiftRuntime.reset();

            boolean isStall = false;

            int currPos = -1;
            while (OpModeUtils.opModeIsActive() && depoLift.isBusy()) {
                if(currPos == -1 || depoLiftRuntime.seconds() > 1) {
                    if(currPos != -1 && currPos == depoLift.getCurrentPosition()) {
                        isStall = true;
                        break;
                    }

                    currPos = depoLift.getCurrentPosition();
                    depoLiftRuntime.reset();
                }
            }

            depoLift.setPower(0);
            depo.setPosition(depoLowerPos);
        } else {
            flipCollectorBox(intakeFlipPrepPos);
            MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, depoLift);
            depoLift.setTargetPosition(targetPosition);
            depoLift.setPower(1);

            runtime.reset();
            while (OpModeUtils.opModeIsActive() && depoLift.isBusy() && runtime.seconds() < 2) {
                // wait for motor to stop
                if(depoLift.getCurrentPosition() > 300) {
                    depo.setPosition(0.3);
                }
            }
        }
    }

    private void rotateDepoBox(double finalPosition) {
        if(depo.getPosition() < finalPosition) {
            depo.setPosition(finalPosition);
        }
    }
}
