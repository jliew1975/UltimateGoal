package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import android.content.res.AssetManager;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@Autonomous(name="TestingPersonal", group="Group 1")
public class TestingApp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        Trajectory trajectory = AssetsTrajectoryManager.load("TestingBlue");

        SkyStoneAutoRobot robot = new SkyStoneAutoRobot();
        robot.init_old_drive();

        robot.drive.followTrajectorySync(trajectory);
    }
}
