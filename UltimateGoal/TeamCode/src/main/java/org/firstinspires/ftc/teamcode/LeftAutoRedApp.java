package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.detectors.StarterRingsDetector.RingCount;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.AutonomousColor;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@Autonomous(name="Red (L)", group="Group 1")
public class LeftAutoRedApp extends AutoApp {
    private Pose2d startPose = new Pose2d(-62, -26, Math.toRadians(180));

    public LeftAutoRedApp() {
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
                        .splineToSplineHeading(new Pose2d(20, -40, Math.toRadians(90)), Math.toRadians(-90))
                        .build();

        robot.getDrive().followTrajectory(toZoneA1);
        robot.dropWobbleGoal(false);

        if(isPickupSecondWobbleGoal) {
            Trajectory toPickupSecondWobble =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), Math.toRadians(90), DriveConstants.SLOW_CONSTRAINTS)
                            .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                            .splineToLinearHeading(new Pose2d(-33, -53, 0), Math.toRadians(180))
                            .build();

            robot.getDrive().followTrajectory(toPickupSecondWobble);
            robot.pickupWobbleGoal();
            robot.getDrive().turn(Math.toRadians(-180));

            Trajectory toZoneA2 =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), true)
                            .splineToLinearHeading(new Pose2d(-10, -55, Math.toRadians(180)), Math.toRadians(-15))
                            .build();

            Trajectory forward = robot.getDrive().trajectoryBuilder(toZoneA2.end())
                    .forward(10)
                    .build();

            robot.getDrive().followTrajectory(toZoneA2);
            robot.dropWobbleGoal();
            robot.getDrive().followTrajectory(forward);
        }

        ThreadUtils.getExecutorService().submit(() -> {
            prepareShooter();
        });

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-5, -40, 0))
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
                        .splineToSplineHeading(new Pose2d(30, -24, Math.toRadians(145)), Math.toRadians(-35))
                        .build();

        robot.getDrive().followTrajectory(toZoneB1);
        robot.dropWobbleGoal(false);

        if(isPickupSecondWobbleGoal) {
            Trajectory toPickupSecondWobble =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), Math.toRadians(170))
                            .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                            .splineToLinearHeading(new Pose2d(-33, -53, Math.toRadians(-15)), Math.toRadians(180))
                            .build();

            robot.getDrive().followTrajectory(toPickupSecondWobble);
            robot.pickupWobbleGoal();
            robot.getDrive().turn(Math.toRadians(-175));

            Trajectory toZoneB2 =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), true)
                            .splineToSplineHeading(new Pose2d(20, -50, Math.toRadians(-135)), 0)
                            .build();

            Trajectory forward =
                    robot.getDrive().trajectoryBuilder(toZoneB2.end())
                        .forward(5)
                        .build();

            robot.getDrive().followTrajectory(toZoneB2);
            robot.dropWobbleGoal();
            robot.getDrive().followTrajectory(forward);
        }

        ThreadUtils.getExecutorService().submit(() -> {
            prepareShooter();
        });

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), false, DriveConstants.SLOW_CONSTRAINTS)
                        .addDisplacementMarker(() -> prepareShooter())
                        .lineToLinearHeading(new Pose2d(-5, -40, 0))
                        .build();

        robot.getDrive().followTrajectory(toLaunchZone);
        shootPowerShot();

        Trajectory toParking =
                robot.getDrive().trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, -40, 0))
                        .build();

        robot.getDrive().followTrajectory(toParking);
    }

    private void performFour() {
        Trajectory toZoneC1 =
                robot.getDrive().trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(60, -38, Math.toRadians(90)), Math.toRadians(-70))
                        .build();

        robot.getDrive().followTrajectory(toZoneC1);
        robot.dropWobbleGoal();

        if(isPickupSecondWobbleGoal) {
            Trajectory toPickupSecondWobble =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                            .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                            .splineToLinearHeading(new Pose2d(-32, -51, 0), Math.toRadians(180))
                            .build();

            robot.getDrive().followTrajectory(toPickupSecondWobble);
            robot.pickupWobbleGoal();
            robot.getDrive().turn(Math.toRadians(-175));

            Trajectory toZoneC2 =
                    robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate(), true)
                        .lineToLinearHeading(new Pose2d(40, -58, Math.toRadians(180)))
                        .build();

            Trajectory forward =
                    robot.getDrive().trajectoryBuilder(toZoneC2.end())
                        .forward(20)
                        .build();

            robot.getDrive().followTrajectory(toZoneC2);
            robot.dropWobbleGoal(true);
            robot.getDrive().followTrajectory(forward);
        }

        ThreadUtils.getExecutorService().submit(() -> {
            prepareShooter();
        });

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(robot.getDrive().getPoseEstimate())
                        .addDisplacementMarker(() -> prepareShooter())
                        .lineToLinearHeading(new Pose2d(-5, -40, 0))
                        .build();

        robot.getDrive().followTrajectory(toLaunchZone);
        shootPowerShot();

        Trajectory toParking =
                robot.getDrive().trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, -40, 0))
                        .build();

        robot.getDrive().followTrajectory(toParking);
    }
}
