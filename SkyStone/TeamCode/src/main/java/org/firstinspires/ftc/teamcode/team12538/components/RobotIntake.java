package org.firstinspires.ftc.teamcode.team12538.components;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

import lombok.Data;

@Data
public class RobotIntake implements RobotComponent, ControlAware {
    private DcMotorWrapper leftRoller;
    private DcMotorWrapper rightRoller;

    private volatile boolean isStuck = false;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();

        leftRoller =  new DcMotorWrapper("leftRollerIntake", hardwareMap);
        rightRoller = new DcMotorWrapper("rightRollerIntake", hardwareMap);

        leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, leftRoller, rightRoller);
    }

    @Override
    public void control(Gamepad gamepad) {
        double power = 1d;

        if(gamepad.right_bumper) {
            setPower(power);
            OpModeUtils.getGlobalStore().setLiftOuttake(true);
        } else if(gamepad.left_bumper) {
            setPower(-1 * power);
            OpModeUtils.getGlobalStore().setLiftOuttake(true);
        } else {
            OpModeUtils.getGlobalStore().setLiftOuttake(false);
            setPower(0d);
        }
    }

    public void setPower(double power) {
        leftRoller.setPower(power);
        rightRoller.setPower(power);

        if(power > 0 && OpModeUtils.getGlobalStore().runMode == OpModeStore.RunMode.Autonomous) {
            ThreadUtils.getExecutorService().submit(() -> {
                while (OpModeUtils.opModeIsActive() && leftRoller.getPower() > 0 && rightRoller.getPower() > 0) {
                    double currentDraw =
                            Math.min(leftRoller.getCurrentPowerDraw(), rightRoller.getCurrentPowerDraw());

                    isStuck = currentDraw > 3.5;
                }
            });
        } else if(power < 0) {
            isStuck = false;
        }
    }
}
