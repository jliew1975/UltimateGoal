package org.firstinspires.ftc.teamcode.team12538.opModes.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.team12538.utils.AssetsTrajectoryLoader;

import java.io.IOException;

/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous(name="RR Test", group = "Test")
public class RoadRunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        /*
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30, 5, 0))
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(60, -5, 0))
                        .build()
        );

drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(40, 15))
                        .build()
        );
         */


        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(34, 5, Math.toRadians(45)))
                        .build()
        );

        /*
        try {
            Trajectory trajectory = AssetsTrajectoryLoader.load("Test");
            drive.followTrajectorySync(trajectory);
        } catch (Exception e) {
            e.printStackTrace();
        }
        */
    }
}
