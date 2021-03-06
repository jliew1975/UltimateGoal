package org.firstinspires.ftc.teamcode.team12538.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.models.OpModeStore;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class OpModeUtils {
    private static ElapsedTime ELAPSED_TIME = new ElapsedTime();

    private static OpModeStore GLOBAL_STORE = new OpModeStore();

    public static LinearOpMode getOpMode() {
        return getGlobalStore().getOpMode();
    }

    public static void init(LinearOpMode opMode) {
        ThreadUtils.init();
        getGlobalStore().setOpMode(opMode);
        getGlobalStore().setHardwareMap(opMode.hardwareMap);
        getGlobalStore().setTelemetry(opMode.telemetry);
    }

    public static OpModeStore getGlobalStore() {
        return GLOBAL_STORE;
    }

    public static boolean opModeIsActive() {
        return getOpMode().opModeIsActive();
    }

    public static void stop() {
        getGlobalStore().clear();
        ThreadUtils.shutdown();
    }

    public static boolean isDisableInitPos() {
        return getGlobalStore().isDisableInitPos();
    }

    public static void stampStartTime() {
        ELAPSED_TIME.reset();
    }

    public static long getTimeRemaining() {
        return 30 - Math.round(ELAPSED_TIME.milliseconds() + 0.5);
    }
}
