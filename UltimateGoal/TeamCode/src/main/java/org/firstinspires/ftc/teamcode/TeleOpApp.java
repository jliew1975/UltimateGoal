package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.components.TargetConstant;
import org.firstinspires.ftc.teamcode.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.robot.TeleOpRobot;
import org.firstinspires.ftc.teamcode.util.OpModeStore;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Robot Tele", group="Group 1")
public class TeleOpApp extends LinearOpMode {

    public TeleOpRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Tell global store that runMode is in Autonomous mode
        OpModeUtils.getGlobalStore().runMode = OpModeStore.RunMode.TeleOp;

        // Reset encoder
        OpModeUtils.setResetEncoder(true);

        // Init ThreadUtils
        ThreadUtils.init();

        // Init OpModeUtils
        OpModeUtils.init(this);

        robot = new TeleOpRobot();
        robot.init();

        waitForStart();

        if(isStopRequested()) {
            return;
        }

        while(opModeIsActive() && !isStopRequested()) {
            robot.controlA(gamepad1);
            robot.controlB(gamepad2);

            telemetry.update();
        }
    }


}
