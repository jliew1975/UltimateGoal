package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.detectors.StarterRingsDetector.RingCount;
import org.firstinspires.ftc.teamcode.util.AutonomousColor;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@Autonomous(name="Blue (R)", group="Group 1")
public class RightAutoBlueApp extends AutoApp {
    private Pose2d startPose = new Pose2d(-63, 25, Math.toRadians(180));

    public RightAutoBlueApp() {
        super.autoColor = AutonomousColor.Blue;
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
                robot.getDrive().trajectoryBuilder(startPose, Math.toRadians(-15))
                        .splineToLinearHeading(new Pose2d(12, 45, Math.toRadians(-90)), Math.toRadians(90))
                        .build();

        robot.getDrive().followTrajectory(toZoneA1);
        robot.dropWobbleGoal();

        if(isPickupSecondWobbleGoal) {
            Trajectory toPickupSecondWobble =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), Math.toRadians(80))
                            .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                            .splineToLinearHeading(new Pose2d(-39, 56, Math.toRadians(30)), Math.toRadians(-140))
                            .build();

            robot.getDrive().followTrajectory(toPickupSecondWobble);
            robot.pickupWobbleGoal();
            robot.getDrive().turn(Math.toRadians(180));

            Trajectory toZoneA2 =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-5, 60, Math.toRadians(180)))
                            .build();

            robot.getDrive().followTrajectory(toZoneA2);
            robot.dropWobbleGoal();
        }

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .addDisplacementMarker(() -> prepareShooter())
                        .lineToLinearHeading(new Pose2d(-5, 40, 0))
                        .build();

        robot.getDrive().followTrajectory(toLaunchZone);
        shootPowerShot();

        Trajectory toParking =
                robot.getDrive().trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, 10, 0))
                        .build();

        robot.getDrive().followTrajectory(toParking);
    }


    private void performOne() {
        Trajectory toZoneB1 =
                robot.getDrive().trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(26, 25, Math.toRadians(35)), Math.toRadians(-135))
                        .build();

        robot.getDrive().followTrajectory(toZoneB1);
        robot.dropWobbleGoal();

        if(isPickupSecondWobbleGoal) {
            Trajectory toPickupSecondWobble =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                            .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                            .splineToLinearHeading(new Pose2d(-35, 50, 0), Math.toRadians(-170))
                            .build();

            robot.getDrive().followTrajectory(toPickupSecondWobble);
            robot.pickupWobbleGoal();
            robot.getDrive().turn(180);

            Trajectory toZoneB2 =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                            .splineToLinearHeading(new Pose2d(25, 47, Math.toRadians(145)), Math.toRadians(-40))
                            .build();

            robot.getDrive().followTrajectory(toZoneB2);
            robot.dropWobbleGoal();
        }

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .addDisplacementMarker(() -> prepareShooter())
                        .lineToLinearHeading(new Pose2d(-5, -40, 0))
                        .build();

        robot.getDrive().followTrajectory(toLaunchZone);
        shootPowerShot();

        Trajectory toParking =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(10, 10, 0))
                        .build();

        robot.getDrive().followTrajectory(toParking);
    }

    private void performFour() {
        Trajectory toZoneC1 =
                robot.getDrive().trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(60, -45, Math.toRadians(90)), Math.toRadians(-90))
                        .build();

        robot.getDrive().followTrajectory(toZoneC1);
        robot.dropWobbleGoal();

        if(isPickupSecondWobbleGoal) {
            Trajectory toPickupSecondWobble =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                            .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                            .lineToLinearHeading(new Pose2d(-38, -50, 0))
                            .build();

            robot.getDrive().followTrajectory(toPickupSecondWobble);
            robot.pickupWobbleGoal();

            Trajectory toZoneC2 =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-10, -60, Math.toRadians(180)), 0)
                        .lineToLinearHeading(new Pose2d(45, -60, Math.toRadians(180)))
                        .build();

            robot.getDrive().followTrajectory(toZoneC2);
            robot.dropWobbleGoal();
        }

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .addDisplacementMarker(() -> prepareShooter())
                        .lineToLinearHeading(new Pose2d(-5, -40, 0))
                        .build();

        robot.getDrive().followTrajectory(toLaunchZone);
        shootPowerShot();

        Trajectory toParking =
                robot.getDrive().trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, -10, 0))
                        .build();

        robot.getDrive().followTrajectory(toParking);
    }
}
