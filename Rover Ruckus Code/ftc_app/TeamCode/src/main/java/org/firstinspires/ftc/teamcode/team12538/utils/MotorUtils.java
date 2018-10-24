package org.firstinspires.ftc.teamcode.team12538.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

public class MotorUtils {
    public static void setMode(DcMotor.RunMode mode, DcMotor... motors) {
        for(DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public static void setMode(DcMotor.RunMode mode, List<DcMotor> motors) {
        for(DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public static void setZeroPowerMode(DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotor... motors) {
        for(DcMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public static void setZeroPowerMode(DcMotor.ZeroPowerBehavior zeroPowerBehavior, List<DcMotor> motors) {
        for(DcMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }
}
