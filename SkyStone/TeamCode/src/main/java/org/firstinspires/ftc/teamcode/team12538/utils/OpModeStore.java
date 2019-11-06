package org.firstinspires.ftc.teamcode.team12538.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OpModeStore {
    public enum RunMode { Autonomous, TeleOp }

    private LinearOpMode opMode = null;
    public RunMode runMode = RunMode.TeleOp;

    private volatile boolean resetEncoder = false;
    private volatile boolean foundationClawDown = false;

    private volatile boolean liftOuttake = false;

    public void init() {
        resetEncoder = false;
        foundationClawDown = false;
    }

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void destroy() {
        this.opMode = null;
    }

    public boolean isResetEncoder() {
        return resetEncoder;
    }

    public void setResetEncoder(boolean resetEncoder) {
        this.resetEncoder = resetEncoder;
    }

    public boolean isFoundationClawDown() {
        return foundationClawDown;
    }

    public void setFoundationClawDown(boolean foundationClawDown) {
        this.foundationClawDown = foundationClawDown;
    }

    public boolean isLiftOuttake() {
        return liftOuttake;
    }

    public void setLiftOuttake(boolean liftOuttake) {
        this.liftOuttake = liftOuttake;
    }
}
