package org.firstinspires.ftc.teamcode.team12538.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.team12538.models.OpModeStore;

public class OpModeUtils {
    private static OpModeStore GLOBAL_STORE = new OpModeStore();

    public static LinearOpMode getOpMode() {
        return getGlobalStore().getOpMode();
    }

    public static OpModeStore getGlobalStore() {
        return GLOBAL_STORE;
    }

    public static boolean opModeIsActive() {
        return getOpMode().opModeIsActive();
    }
}
