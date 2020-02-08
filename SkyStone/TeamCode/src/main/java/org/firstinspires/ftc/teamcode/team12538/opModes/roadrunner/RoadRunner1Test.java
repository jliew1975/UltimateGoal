package org.firstinspires.ftc.teamcode.team12538.opModes.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import kotlin.Unit;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name="A RR Test 1", group = "Test")
public class RoadRunner1Test extends LinearOpMode {
    public static int MODE = 0;
    public static double X = 40;
    public static double Y = 20;
    public static double HEADING = 90;
    public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        switch(MODE) {
            case 0:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .splineTo(new Pose2d(X, Y, Math.toRadians(HEADING)))
                                .build()
                );

                break;
            case 1:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(X, Y))
                                .build()
                );

                break;
            case 2:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .forward(DISTANCE)
                                .build()
                );

                break;
            case 3:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .back(DISTANCE)
                                .build()
                );
                break;
            case 4:
                drive.turnSync(Math.toRadians(HEADING));
                break;
            case 5:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(X, Y), new LinearInterpolator(0d, 0d))
                        .build()
                );

        }
    }
}
