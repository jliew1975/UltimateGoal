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
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.io.IOException;

import kotlin.Unit;

/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous(name="RR Test", group = "Test")
public class RoadRunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(60, 0))
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(60, 0))
                        .build()
        );

        /*
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(72 + drive.getLastError().getX(), 0))
                        .build()
        );
        */
    }
}
