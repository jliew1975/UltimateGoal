package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public abstract class RobotApp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        performRobotOperation();
    }

    // Uncomment below line for Motorola E4 phones.
    // Below code is a workaround to prevent driver station disconnect with
    // robot controller due to communication timeout issue
    /*
    @Override
    public void waitForStart() {
        // Do not use waitForStart() if you have Motorola E4 phones.
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
    }
    */

    public abstract void performRobotOperation() throws InterruptedException;
}
