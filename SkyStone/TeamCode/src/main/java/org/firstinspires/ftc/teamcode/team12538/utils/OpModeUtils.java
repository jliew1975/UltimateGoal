package org.firstinspires.ftc.teamcode.team12538.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OpModeUtils {
    private static ElapsedTime ELAPSED_TIME = new ElapsedTime();

    private static OpModeStore GLOBAL_STORE = new OpModeStore();

    public static LinearOpMode getOpMode() {
        return getGlobalStore().getOpMode();
    }

    public static void init(LinearOpMode opMode) {
        ThreadUtils.init();
        getGlobalStore().setOpMode(opMode);
    }

    public static OpModeStore getGlobalStore() {
        return GLOBAL_STORE;
    }

    public static HardwareMap getHardwareMap() {
        return getGlobalStore().getOpMode().hardwareMap;
    }

    public static boolean opModeIsActive() {
        return getOpMode().opModeIsActive();
    }

    public static void stop() {
        getGlobalStore().destroy();
        ThreadUtils.shutdown();
    }

    public static void stampStartTime() {
        ELAPSED_TIME.reset();
    }

    public static long getTimeRemaining() {
        return 30 - Math.round(ELAPSED_TIME.milliseconds() + 0.5);
    }
}
