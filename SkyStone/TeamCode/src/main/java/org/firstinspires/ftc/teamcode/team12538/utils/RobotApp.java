package org.firstinspires.ftc.teamcode.team12538.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class RobotApp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        performRobotOperation();
    }

    public abstract void performRobotOperation() throws InterruptedException;
}
