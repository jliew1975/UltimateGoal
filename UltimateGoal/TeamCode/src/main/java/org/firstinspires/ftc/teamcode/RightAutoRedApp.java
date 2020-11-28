package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.detectors.StarterRingsDetector.RingCount;
import org.firstinspires.ftc.teamcode.util.AutonomousColor;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@Autonomous(name="Red (R)", group="Group 1")
public class RightAutoRedApp extends AutoApp {
    private Pose2d startPose = new Pose2d(-63, -50, Math.toRadians(180));

    public RightAutoRedApp() {
        super.autoColor = AutonomousColor.Red;
    }

    @Override
    public void performRobotOperation() {
        // set robot initial pose
        robot.getDrive().setPoseEstimate(startPose);

        detector.deactivate();
        RingCount ringCount = detector.getRingCount();

        performAutoLogic(ringCount);

        ThreadUtils.sleep(3000);
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
        Trajectory toZoneA1 =
                robot.getDrive().trajectoryBuilder(startPose, true)
                        .splineToLinearHeading(new Pose2d(-5, -60, Math.toRadians(180)), 0)
                        .build();

        robot.getDrive().followTrajectory(toZoneA1);
        robot.dropWobbleGoal();

        if(isPickupSecondWobbleGoal) {
            Trajectory toSecondWobble =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-38, -25, 0), Math.toRadians(180))
                            .build();

            robot.getDrive().followTrajectory(toSecondWobble);
            robot.pickupWobbleGoal();

            Trajectory toZoneA2 =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                            .splineToLinearHeading(new Pose2d(10, -45, Math.toRadians(90)), 0)
                            .build();

            robot.getDrive().followTrajectory(toZoneA2);
            robot.dropWobbleGoal();
        }

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(-5, -40, 0))
                        .build();

        robot.getDrive().followTrajectory(toLaunchZone);
        shootPowerShot();

        Trajectory toParking =
                robot.getDrive().trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, -40, 0))
                        .build();

        robot.getDrive().followTrajectory(toParking);
    }


    private void performOne() {
        Trajectory toZoneB1 =
                robot.getDrive().trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(20, -48, Math.toRadians(-135)), Math.toRadians(35))
                        .addDisplacementMarker(() -> robot.prepareWobbleArm() )
                        .build();

        robot.getDrive().followTrajectory(toZoneB1);
        robot.dropWobbleGoal();

        if(isPickupSecondWobbleGoal) {
            Trajectory toSecondWobble =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), Math.toRadians(140))
                            .splineToLinearHeading(new Pose2d(-40, -22, Math.toRadians(35)), Math.toRadians(140))
                            .build();

            robot.getDrive().followTrajectory(toSecondWobble);
            robot.pickupWobbleGoal();

            Trajectory toZoneB2 =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), Math.toRadians(30))
                            .splineToLinearHeading(new Pose2d(23, -25, Math.toRadians(135)), Math.toRadians(-30))
                            .build();

            robot.getDrive().followTrajectory(toZoneB2);
            robot.dropWobbleGoal();
        }

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(-5, -40, 0))
                        .build();

        robot.getDrive().followTrajectory(toLaunchZone);
        shootPowerShot();

        Trajectory toParking =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(10, -40, 0))
                        .build();

        robot.getDrive().followTrajectory(toParking);
    }

    private void performFour() {
        Trajectory toZoneC1 =
                robot.getDrive().trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(40, -60, Math.toRadians(180)), 0)
                        .addDisplacementMarker(() -> robot.prepareWobbleArm() )
                        .build();

        robot.getDrive().followTrajectory(toZoneC1);
        robot.dropWobbleGoal();

        if(isPickupSecondWobbleGoal) {
            Trajectory toSecondWobble =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), Math.toRadians(140))
                            .splineToLinearHeading(new Pose2d(-40, -22, Math.toRadians(35)), Math.toRadians(-160))
                            .build();

            robot.getDrive().followTrajectory(toSecondWobble);
            robot.pickupWobbleGoal();

            Trajectory toZoneC2 =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), Math.toRadians(30))
                            .splineToLinearHeading(new Pose2d(50, -50, Math.toRadians(135)), Math.toRadians(-40))
                            .build();

            robot.getDrive().followTrajectory(toZoneC2);
            robot.dropWobbleGoal();
        }

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-5, -40, 0))
                        .build();

        robot.getDrive().followTrajectory(toLaunchZone);
        shootPowerShot();

        Trajectory toParking =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(10, -40, Math.toRadians(0)))
                        .build();

        robot.getDrive().followTrajectory(toParking);
    }
}
