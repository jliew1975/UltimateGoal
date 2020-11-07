package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareUtils {
    public static DcMotor get(String name, HardwareMap hardwareMap) {
        return hardwareMap.get(DcMotor.class, name);
    }
}
