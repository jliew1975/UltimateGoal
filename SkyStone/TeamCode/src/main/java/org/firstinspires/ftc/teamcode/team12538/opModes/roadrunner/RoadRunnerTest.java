package org.firstinspires.ftc.teamcode.team12538.opModes.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.team12538.utils.AssetsTrajectoryLoader;

import java.io.IOException;

import kotlin.Unit;

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

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(25, 35, Math.toRadians(90)))
                        .build()
        );

        sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(18, 86, Math.toRadians(180)))
                .build()
        );

        sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(20)
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(15)
                        .build()
        );

        drive.turnSync(Math.toRadians(90));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(20, 35, Math.toRadians(-90)))
                        .build()
        );

        /*
        sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(13)
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(10, 86), new SplineInterpolator(Math.toRadians(180), Math.toRadians(-90)))
                        .build()
        );

        /*
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(30)
                        .build()
        );

        drive.turnSync(Math.toRadians(90));

        sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(25, 20, Math.toRadians(-90)))
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
