package org.firstinspires.ftc.teamcode.team12538.utils;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class ThreadUtils {
    private static ExecutorService executorService = null;

    public static void init() {
        executorService = Executors.newSingleThreadExecutor();
    }

    public static void sleep(long time) {
        try {
            TimeUnit.MILLISECONDS.sleep(time);
        } catch(InterruptedException ie) {
            Thread.currentThread().interrupt();
        }
    }

    public static void idle() {
        Thread.yield();
    }

    public static ExecutorService getExecutorService() {
        return executorService;
    }

    public static void shutdown() {
        executorService.shutdownNow();
    }
}