package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.List;

public class MotorUtils {
    public static void setMode(DcMotor.RunMode mode, DcMotor... motors) {
        setMode(mode, Arrays.asList(motors));
    }

    public static void setMode(DcMotor.RunMode mode, List<DcMotor> motors) {
        motors.stream().forEach(motor -> motor.setMode(mode));
    }

    public static void setZeroPowerMode(DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotor... motors) {
        setZeroPowerMode(zeroPowerBehavior, Arrays.asList(motors));
    }

    public static void setZeroPowerMode(DcMotor.ZeroPowerBehavior zeroPowerBehavior, List<DcMotor> motors) {
        motors.stream().forEach(motor -> motor.setZeroPowerBehavior(zeroPowerBehavior));
    }

    public static boolean motorIsBusy(DcMotor... motors) {
        return motorIsBusy(Arrays.asList(motors));
    }

    public static boolean motorIsBusy(List<DcMotor> motors) {
        if(motors.isEmpty()) {
            return false;
        }

        return !motors.stream().filter(motor -> !motor.isBusy()).findFirst().isPresent();
    }
}
