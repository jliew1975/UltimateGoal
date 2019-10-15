package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class RobotIntake implements RobotComponent {
    DcMotorWrapper leftRoller;
    DcMotorWrapper rightRoller;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getOpMode().hardwareMap;

        leftRoller =  new DcMotorWrapper("leftRollerIntake", hardwareMap);
        rightRoller = new DcMotorWrapper("rightRollerIntake", hardwareMap);

        leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, leftRoller, rightRoller);
    }

    public void setPower(double power) {
        leftRoller.setPower(power);
        rightRoller.setPower(power);
    }

    @Override
    public void printTelemetry() {
        // intensionally left blank
    }
}
