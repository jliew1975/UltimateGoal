package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public abstract class RobotApp extends LinearOpMode {
    public AutonomousMode autoMode = AutonomousMode.Unknown;
    public AutonomousColor autoColor = AutonomousColor.Unknown;

    protected SkyStoneAutoRobot robot = null;
    protected AutoGamepad gamepad = null;

    protected int numSkystone = 3;

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
            case CurveRight:
                return MecanumDrive.AutoDirection.CurveLeft;
            case CurveLeft:
                return MecanumDrive.AutoDirection.CurveRight;
            case StrafeLeft:
                return MecanumDrive.AutoDirection.StrafeRight;
            default:
                return MecanumDrive.AutoDirection.StrafeLeft;
        }
    }

    protected void deployStone(int height) {
        robot.outtake.prepareForStoneDeployment();
        robot.outtake.outtakeSlides.runToPosition(height, true);

        robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
        sleep(200);
        robot.outtake.outtakeSlides.runToStoneHeight(robot.outtake.stoneHeight);
        sleep(200);
        robot.outtake.performStoneIntakeOperation();
    }
}
