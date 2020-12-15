package org.firstinspires.ftc.teamcode.trajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.detectors.StarterRingsDetector.RingCount;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.GlobalStorage;
import org.firstinspires.ftc.teamcode.util.ShooterUtils;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TrajectoryBlueLeftFactory {
    public static Map<RingCount, List<TrajectoryWrapper>> generateTrajectories(AutoRobot robot) {
        Map<RingCount, List<TrajectoryWrapper>> trajectoryMap = new HashMap<>();

        Pose2d startPose = new Pose2d(-63.0, 50.0, Math.toRadians(180));

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
                        .lineToLinearHeading(new Pose2d(5.0, 60.0, Math.toRadians(180)))
                        .build();

        Trajectory forward1 =
                drive.trajectoryBuilder(toZone1.end(), false)
                        .forward(10.0)
                        .build();

        Trajectory turn1 =
                drive.trajectoryBuilder(forward1.end(), false)
                        .lineToLinearHeading(new Pose2d(-10.0, 45.0, Math.toRadians(15)))
                        .build();

        Trajectory toPickupSecondWobble =
                drive.trajectoryBuilder(turn1.end(), false)
                        .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                        .lineToLinearHeading(new Pose2d(-38.0, 28.0, 0.0))
                        .build();

        Trajectory toZone2 =
                drive.trajectoryBuilder(toPickupSecondWobble.end(), true)
                        .splineToLinearHeading(new Pose2d(-5.0, 60.0, Math.toRadians(180)), Math.toRadians(0.0))
                        .build();

        Trajectory forward2 =
                drive.trajectoryBuilder(toZone2.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(10)
                        .build();

        Trajectory toLaunchZone =
                drive.trajectoryBuilder((GlobalStorage.wobbleCount > 1) ? forward2.end() : forward1.end())
                        .lineToLinearHeading(new Pose2d(-5, 40, 0))
                        .build();

        Trajectory toParking =
                drive.trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, 40, 0))
                        .build();

        return Arrays.asList(
                new TrajectoryWrapper(toZone1),
                new TrajectoryWrapper(forward1),
                new TrajectoryWrapper(turn1),
                new TrajectoryWrapper(toPickupSecondWobble),
                new TrajectoryWrapper(toZone2),
                new TrajectoryWrapper(forward2),
                new TrajectoryWrapper(toLaunchZone),
                new TrajectoryWrapper(toParking));
    }

    private static List<TrajectoryWrapper> buildOne(AutoRobot robot, Pose2d startPose) {
        SampleMecanumDrive drive = robot.getDrive();

        Trajectory toZone1 =
                drive.trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(30.0, 50.0, Math.toRadians(160)), Math.toRadians(-10))
                        .build();

        Trajectory forward1 =
                drive.trajectoryBuilder(toZone1.end())
                        .forward(10)
                        .build();

        Trajectory turn1 =
                drive.trajectoryBuilder(forward1.end(), false)
                        .lineToLinearHeading(new Pose2d(15.0, 30.0, Math.toRadians(0.0)))
                        .build();

        Trajectory toPickupSecondWobble =
                drive.trajectoryBuilder(turn1.end(), Math.toRadians(180))
                        .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                        .splineToLinearHeading(new Pose2d(-35.0, 23.0, Math.toRadians(-15)), Math.toRadians(140))
                        .build();

        Trajectory reverse =
                drive.trajectoryBuilder(toPickupSecondWobble.end(), false)
                        .lineToLinearHeading(new Pose2d(-25.0, 23.0, Math.toRadians(0.0)))
                        .build();

        Trajectory toZone2 =
                drive.trajectoryBuilder(reverse.end(), false)
                        .splineToSplineHeading(new Pose2d(18.0, 38.0, Math.toRadians(180)), 0.0)
                        .build();

        Trajectory forward2 =
                drive.trajectoryBuilder(toZone2.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(10.0)
                        .build();

        Trajectory toLaunchZone =
                drive.trajectoryBuilder((GlobalStorage.wobbleCount > 1) ? forward2.end() : toZone1.end())
                        .lineToLinearHeading(new Pose2d(-5.0, 40.0, 0))
                        .build();

        Trajectory toParking =
                drive.trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10.0, 40.0, 0))
                        .build();

        return Arrays.asList(
                new TrajectoryWrapper(toZone1),
                new TrajectoryWrapper(forward1),
                new TrajectoryWrapper(turn1),
                new TrajectoryWrapper(toPickupSecondWobble),
                new TrajectoryWrapper(reverse),
                new TrajectoryWrapper(toZone2),
                new TrajectoryWrapper(forward2),
                new TrajectoryWrapper(toLaunchZone),
                new TrajectoryWrapper(toParking));
    }

    private static List<TrajectoryWrapper> buildFour(AutoRobot robot, Pose2d startPose) {
        SampleMecanumDrive drive = robot.getDrive();

        Trajectory toZone1 =
                drive.trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(50.0, 58.0, Math.toRadians(-150.0)), Math.toRadians(10.0))
                        .build();

        Trajectory forward1 =
                drive.trajectoryBuilder(toZone1.end())
                        .forward(10)
                        .build();

        Trajectory turn1 =
                drive.trajectoryBuilder(forward1.end())
                        .lineToLinearHeading(new Pose2d(15.0, 30.0, Math.toRadians(20.0)))
                        .build();

        Trajectory toPickupSecondWobble =
                drive.trajectoryBuilder(turn1.end(), Math.toRadians(-130.0))
                        .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                        .splineToLinearHeading(new Pose2d(-35.0, 23.0, Math.toRadians(-18.0)), Math.toRadians(140))
                        .build();

        Trajectory turn2 =
                drive.trajectoryBuilder(forward1.end())
                        .lineToLinearHeading(new Pose2d(-25.0, 20.0, Math.toRadians(180.0)))
                        .build();

        Trajectory toZone2 =
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(48.0, 48.0, Math.toRadians(-110)), Math.toRadians(30.0))
                        .build();

        Trajectory forward2 =
                drive.trajectoryBuilder(toZone2.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(10)
                        .build();

        Trajectory toLaunchZone =
                drive.trajectoryBuilder((GlobalStorage.wobbleCount > 1) ? forward2.end() : forward1.end())
                        .lineToLinearHeading(new Pose2d(-5, 40, 0))
                        .build();

        Trajectory toParking =
                drive.trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, 40, 0))
                        .build();

        return Arrays.asList(
                new TrajectoryWrapper(toZone1),
                new TrajectoryWrapper(forward1),
                new TrajectoryWrapper(turn1),
                new TrajectoryWrapper(toPickupSecondWobble),
                new TrajectoryWrapper(turn2),
                new TrajectoryWrapper(toZone2),
                new TrajectoryWrapper(forward2),
                new TrajectoryWrapper(toLaunchZone),
                new TrajectoryWrapper(toParking));
    }
}
