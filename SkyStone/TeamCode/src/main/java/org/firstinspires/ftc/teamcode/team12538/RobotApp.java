package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoColor;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public abstract class RobotApp extends LinearOpMode {
    public AutoColor autoColor = AutoColor.Unknown;

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

    protected MecanumDrive.AutoDirection flipDirection(MecanumDrive.AutoDirection direction) {
        switch(direction) {
            case Backward:
                return MecanumDrive.AutoDirection.Forward;
            case Forward:
                return MecanumDrive.AutoDirection.Backward;
            case TurnRight:
                return MecanumDrive.AutoDirection.TurnLeft;
            case TurnLeft:
                return MecanumDrive.AutoDirection.TurnRight;
            case StrafeLeft:
                return MecanumDrive.AutoDirection.StrafeRight;
            default:
                return MecanumDrive.AutoDirection.StrafeLeft;
        }
    }
}
