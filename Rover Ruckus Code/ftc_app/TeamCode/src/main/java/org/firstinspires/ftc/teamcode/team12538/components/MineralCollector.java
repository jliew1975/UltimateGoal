package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class MineralCollector implements RobotMechanic {
    public enum Direction { Forward, Backward }

    private CRServo intake = null;
    private Servo arm = null;
    private DcMotor armExtension = null;

    private int upperLimit;
    private int lowerLimit;

    private int lowerSlowdownThreashold = 500;
    private int upperSlowdownThreashold = 9500;

    private double slowSpeed = 0.3;

    public MineralCollector(int lowerLimit, int upperLimit) {
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
    }

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        intake = hardwareMap.crservo.get("intake");

        arm = hardwareMap.servo.get("arm");
        arm.setDirection(Servo.Direction.REVERSE);
        arm.setPosition(0.1);

        armExtension = hardwareMap.get(DcMotor.class, "armExtension");
        armExtension.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, armExtension);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, armExtension);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void enableIntake(Direction direction) {
        if(direction == Direction.Forward) {
            intake.setPower(1);
        } else {
            intake.setPower(-1);
        }
    }

    public void disableIntake() {
        intake.setPower(0);
    }

    public void lowerArm() {
        arm.setPosition(0.85);
    }

    public void lowerArm(double power) {
        arm.setPosition(arm.getPosition() + power);
    }

    public void liftArm() {
        arm.setPosition(0.1);
    }

    public void liftArm(double power) {
        arm.setPosition(arm.getPosition() - power);
    }

    public void controlArm(double power) {
        double effectivePower = power;
        if(armExtension != null) {
            int curPos = armExtension.getCurrentPosition();
            if (power > 0 && curPos < upperLimit ) {
                if(curPos >= upperSlowdownThreashold) {
                    effectivePower = slowSpeed;
                }

                armExtension.setPower(effectivePower);
            } else if (power < 0) {
                armExtension.setPower(effectivePower);
            } else {
                armExtension.setPower(0);
            }

            if(curPos > 500) {
                arm.setPosition(1);
                intake.setPower(1);
            }
        }
    }

    public int getLowerSlowdownThreashold() {
        return lowerSlowdownThreashold;
    }

    public void setLowerSlowdownThreashold(int lowerSlowdownThreashold) {
        this.lowerSlowdownThreashold = lowerSlowdownThreashold;
    }

    public int getUpperSlowdownThreashold() {
        return upperSlowdownThreashold;
    }

    public void setUpperSlowdownThreashold(int upperSlowdownThreashold) {
        this.upperSlowdownThreashold = upperSlowdownThreashold;
    }

    public double getSlowSpeed() {
        return slowSpeed;
    }

    public void setSlowSpeed(double slowSpeed) {
        this.slowSpeed = slowSpeed;
    }
}
