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
}
