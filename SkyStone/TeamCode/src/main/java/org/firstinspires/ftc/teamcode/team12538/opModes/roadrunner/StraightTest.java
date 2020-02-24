package org.firstinspires.ftc.teamcode.team12538.opModes.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        Trajectory trajectory1 = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder()
                .back(DISTANCE)
                .build();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory1);

        /*
        while(!isStopRequested()) {
            drive.followTrajectorySync(trajectory1);
            sleep(2000);
            drive.followTrajectorySync(trajectory2);
            sleep(2000);
        }
        */
    }
}
