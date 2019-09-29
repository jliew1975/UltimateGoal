package org.firstinspires.ftc.teamcode.team12538.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.team12538.ext.DcMotorWrapper;

import java.util.Arrays;
import java.util.List;

public class MotorUtils {
    public static void setMode(DcMotor.RunMode mode, DcMotorWrapper... motors) {
        setMode(mode, Arrays.asList(motors));
    }

    public static void setMode(DcMotor.RunMode mode, List<DcMotorWrapper> motors) {
        for(DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public static void setZeroPowerMode(DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotorWrapper... motors) {
        setZeroPowerMode(zeroPowerBehavior, Arrays.asList(motors));
    }

    public static void setZeroPowerMode(DcMotor.ZeroPowerBehavior zeroPowerBehavior, List<DcMotorWrapper> motors) {
        for(DcMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public static boolean motorIsBusy(List<DcMotorWrapper> motors) {
        if(motors.isEmpty()) {
            return false;
        }

        if(motors.size() == 1) {
            return motors.get(0).isBusy();
        } else if(motors.size() == 2) {
            return motors.get(0).isBusy() && motors.get(1).isBusy();
        } else if(motors.size() == 3) {
            return motors.get(0).isBusy() && motors.get(1).isBusy() && motors.get(2).isBusy();
        }

        return motors.get(0).isBusy() && motors.get(1).isBusy() && motors.get(2).isBusy() && motors.get(3).isBusy();
    }
}
