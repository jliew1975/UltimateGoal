package org.firstinspires.ftc.teamcode.team12538.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OpModeStore {
    private LinearOpMode opMode = null;
    private boolean encoderDriveEnabled = false;

    private volatile boolean foundationClawDown = false;

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void destroy() {
        this.opMode = null;
    }

    public boolean isEnableDriveEncoder() {
        return encoderDriveEnabled;
    }

    public void setEncoderDriveEnabled(boolean encoderDriveEnabled) {
        this.encoderDriveEnabled = encoderDriveEnabled;
    }

    public boolean isFoundationClawDown() {
        return foundationClawDown;
    }

    public void setFoundationClawDown(boolean foundationClawDown) {
        this.foundationClawDown = foundationClawDown;
    }
}
