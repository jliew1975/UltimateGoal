package org.firstinspires.ftc.teamcode.trajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.detectors.StarterRingsDetector.RingCount;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.GlobalStorage;
import org.firstinspires.ftc.teamcode.util.Pose2dUtils;
import org.firstinspires.ftc.teamcode.util.ShooterUtils;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TrajectoryRedLeftFactory {
    public static Map<RingCount, List<TrajectoryWrapper>> generateTrajectories(AutoRobot robot) {
        Map<RingCount, List<TrajectoryWrapper>> trajectoryMap = new HashMap<>();

        Pose2d startPose = new Pose2d(-63, -26, Math.toRadians(180));

        robot.getDrive().setPoseEstimate(startPose);
        trajectoryMap.put(RingCount.NONE, buildNone(robot, startPose));
        trajectoryMap.put(RingCount.ONE, buildOne(robot, startPose));
        trajectoryMap.put(RingCount.FOUR, buildFour(robot, startPose));

        return trajectoryMap;
    }

    private static List<TrajectoryWrapper> buildNone(AutoRobot robot, Pose2d startPose) {
        SampleMecanumDrive drive = robot.getDrive();

        Trajectory toZone1 =
                drive.trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(20.0, -40.0, Math.toRadians(90.0)), Math.toRadians(-90.0))
                        .build();

        Trajectory forward1 =
                drive.trajectoryBuilder(toZone1.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(10.0)
                        .build();

        Trajectory toPickupWobble =
                drive.trajectoryBuilder(toZone1.end(), Math.toRadians(90))
                        .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                        .splineToLinearHeading(new Pose2d(-33.0, -53.0, 0.0), Math.toRadians(180.0))
                        .build();

        Trajectory toZone2 =
                drive.trajectoryBuilder(updateHeading(toPickupWobble.end(), -180), true)
                        .splineToLinearHeading(new Pose2d(-10.0, -55.0, Math.toRadians(180.0)), Math.toRadians(-15.0))
                        .build();

        Trajectory forward2 =
                drive.trajectoryBuilder(toZone2.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(10.0)
                        .build();

        Trajectory toLaunchZone =
                drive.trajectoryBuilder((GlobalStorage.wobbleCount > 1) ? forward2.end() : forward1.end())
                        .lineToLinearHeading(new Pose2d(-5.0, -40.0, 0.0))
                        .build();

        Trajectory toParking =
                drive.trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10.0, -40.0, 0.0))
                        .build();

        if(GlobalStorage.wobbleCount > 1) {
            return Arrays.asList(
                    new TrajectoryWrapper(toZone1),
                    new TrajectoryWrapper(forward1),
                    new TrajectoryWrapper(toPickupWobble),
                    new TrajectoryWrapper(-180.0),
                    new TrajectoryWrapper(toZone2),
                    new TrajectoryWrapper(forward2),
                    new TrajectoryWrapper(toLaunchZone),
                    new TrajectoryWrapper(toParking));
        } else {
            return Arrays.asList(
                    new TrajectoryWrapper(toZone1),
                    new TrajectoryWrapper(forward1),
                    new TrajectoryWrapper(toLaunchZone),
                    new TrajectoryWrapper(toParking));
        }
    }

    private static List<TrajectoryWrapper> buildOne(AutoRobot robot, Pose2d startPose) {
        SampleMecanumDrive drive = robot.getDrive();

        Trajectory toZone1 =
                drive.trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(30.0, -24.0, Math.toRadians(145.0)), Math.toRadians(-35.0))
                        .build();

        Trajectory forward1 =
                drive.trajectoryBuilder(toZone1.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(10.0)
                        .build();

        Trajectory toPickupWobble =
                drive.trajectoryBuilder(toZone1.end(), Math.toRadians(170.0))
                        .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                        .splineToLinearHeading(new Pose2d(-33.5, -53.0, Math.toRadians(-15.0)), Math.toRadians(180.0))
                        .build();

        Trajectory toZone2 =
                drive.trajectoryBuilder(updateHeading(toPickupWobble.end(), -175), true)
                        .splineToSplineHeading(new Pose2d(20.0, -50.0, Math.toRadians(-135.0)), 0.0)
                        .build();

        Trajectory forward2 =
                drive.trajectoryBuilder(toZone2.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(10.0)
                        .build();

        Trajectory toLaunchZone =
                drive.trajectoryBuilder((GlobalStorage.wobbleCount > 1) ? forward2.end() : forward1.end())
                        .lineToLinearHeading(new Pose2d(-5.0, -40.0, 0.0))
                        .build();

        Trajectory toParking =
                drive.trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10.0, -40.0, 0.0))
                        .build();

        if(GlobalStorage.wobbleCount > 1) {
            return Arrays.asList(
                    new TrajectoryWrapper(toZone1),
                    new TrajectoryWrapper(forward1),
                    new TrajectoryWrapper(toPickupWobble),
                    new TrajectoryWrapper(-175.0),
                    new TrajectoryWrapper(toZone2),
                    new TrajectoryWrapper(forward2),
                    new TrajectoryWrapper(toLaunchZone),
                    new TrajectoryWrapper(toParking));
        } else {
            return Arrays.asList(
                    new TrajectoryWrapper(toZone1),
                    new TrajectoryWrapper(forward1),
                    new TrajectoryWrapper(toLaunchZone),
                    new TrajectoryWrapper(toParking));
        }
    }

    private static Pose2d updateHeading(Pose2d currPose, double angle) {
        double newHeading = currPose.getHeading() + Math.toRadians(angle);
        return Pose2dUtils.updateHeading(currPose, newHeading);
    }

    private static List<TrajectoryWrapper> buildFour(AutoRobot robot, Pose2d startPose) {
        SampleMecanumDrive drive = robot.getDrive();

        Trajectory toZone1 =
                drive.trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(60.0, -38.0, Math.toRadians(90.0)), Math.toRadians(-70.0))
                        .build();

        Trajectory forward1 =
                drive.trajectoryBuilder(toZone1.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(20)
                        .build();

        Trajectory toPickupWobble =
                drive.trajectoryBuilder(toZone1.end())
                        .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                        .splineToLinearHeading(new Pose2d(-33.0, -51.0, Math.toRadians(-15.0)), Math.toRadians(180.0))
                        .build();

        Trajectory toZone2 =
                drive.trajectoryBuilder(updateHeading(toPickupWobble.end(), -175), true)
                        .lineToLinearHeading(new Pose2d(40.0, -58.0, Math.toRadians(180.0)))
                        .build();

        Trajectory forward2 =
                drive.trajectoryBuilder(toZone2.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(20)
                        .build();

        Trajectory toLaunchZone =
                drive.trajectoryBuilder((GlobalStorage.wobbleCount > 1) ? forward2.end() : forward1.end())
                        .lineToLinearHeading(new Pose2d(-5.0, -40.0, 0.0))
                        .build();

        Trajectory toParking =
                drive.trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10.0, -40.0, 0.0))
                        .build();

        if(GlobalStorage.wobbleCount > 1) {
            return Arrays.asList(
                    new TrajectoryWrapper(toZone1),
                    new TrajectoryWrapper(forward1),
                    new TrajectoryWrapper(toPickupWobble),
                    new TrajectoryWrapper(-175.0),
                    new TrajectoryWrapper(toZone2),
                    new TrajectoryWrapper(forward2),
                    new TrajectoryWrapper(toLaunchZone),
                    new TrajectoryWrapper(toParking));
        } else {
            return Arrays.asList(
                    new TrajectoryWrapper(toZone1),
                    new TrajectoryWrapper(forward1),
                    new TrajectoryWrapper(toLaunchZone),
                    new TrajectoryWrapper(toParking));
        }
    }
}
