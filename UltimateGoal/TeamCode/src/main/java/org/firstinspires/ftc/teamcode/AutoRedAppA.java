package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.detectors.StarterRingsDetector.RingCount;
import org.firstinspires.ftc.teamcode.util.AutonomousColor;

@Autonomous(name="Red (A)", group="Group 1")
public class AutoRedAppA extends AutoApp {
    private Pose2d startPose = new Pose2d(-60, -25, Math.toRadians(0));

    public AutoRedAppA() {
        super.autoColor = AutonomousColor.Red;
    }

    @Override
    public void performRobotOperation() throws InterruptedException {
        // detector.deactivate();
        // RingCount ringCount = detector.getRingCount();

        robot.getDrive().setPoseEstimate(startPose);
        // performAutoLogic(ringCount);
        performAutoLogic(RingCount.FOUR);
    }

    private void performAutoLogic(RingCount ringCount) {
        switch(ringCount) {
            case NONE:
                performNone();
                break;
            case ONE:
                performOne();
                break;
            default:
                performFour();
        }
    }

    private void performNone() {
        Trajectory trajectory =
                robot.getDrive().trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-20, -15), 0)
                    .splineToSplineHeading(new Pose2d(12, -45, Math.toRadians(-90)), Math.toRadians(-90))
                    .build();
        robot.getDrive().followTrajectory(trajectory);
    }


    private void performOne() {
        Trajectory trajectory =
                robot.getDrive().trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(35, -25, Math.toRadians(-90)), Math.toRadians(-90))
                        .build();
        robot.getDrive().followTrajectory(trajectory);
    }

    private void performFour() {
        Trajectory trajectory =
                robot.getDrive().trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(60, -50, Math.toRadians(-90)), Math.toRadians(-90))
                        .build();
        robot.getDrive().followTrajectory(trajectory);
    }
}
