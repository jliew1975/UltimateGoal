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
                        .splineToSplineHeading(new Pose2d(20, -40, Math.toRadians(90)), Math.toRadians(-90))
                        .build();

        Trajectory toPickupSecondWobble =
                drive.trajectoryBuilder(toZone1.end(), Math.toRadians(90))
                        .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                        .splineToLinearHeading(new Pose2d(-33, -53, 0), Math.toRadians(180))
                        .build();

        Trajectory toZone2 =
                drive.trajectoryBuilder(Pose2dUtils.updateHeading(toPickupSecondWobble.end(), 0), true)
                        .splineToLinearHeading(new Pose2d(-10, -55, Math.toRadians(180)), Math.toRadians(-15))
                        .build();

        Trajectory forward2 =
                drive.trajectoryBuilder(toZone2.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(10)
                        .build();

        Trajectory toLaunchZone =
                drive.trajectoryBuilder((GlobalStorage.wobbleCount > 1) ? forward2.end() : toZone1.end())
                        .lineToLinearHeading(new Pose2d(-5, -40, 0))
                        .build();

        Trajectory toParking =
                drive.trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, -40, 0))
                        .build();

        return Arrays.asList(
                new TrajectoryWrapper(toZone1),
                new TrajectoryWrapper(toPickupSecondWobble),
                new TrajectoryWrapper(-180),
                new TrajectoryWrapper(toZone2),
                new TrajectoryWrapper(forward2),
                new TrajectoryWrapper(toLaunchZone),
                new TrajectoryWrapper(toParking));
    }

    private static List<TrajectoryWrapper> buildOne(AutoRobot robot, Pose2d startPose) {
        SampleMecanumDrive drive = robot.getDrive();

        Trajectory toZone1 =
                drive.trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(30, -24, Math.toRadians(145)), Math.toRadians(-35))
                        .build();

        Trajectory toPickupSecondWobble =
                drive.trajectoryBuilder(toZone1.end(), Math.toRadians(170))
                        .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                        .splineToLinearHeading(new Pose2d(-33, -53, Math.toRadians(-15)), Math.toRadians(180))
                        .build();

        Trajectory toZone2 =
                drive.trajectoryBuilder(toPickupSecondWobble.end(), true)
                        .splineToSplineHeading(new Pose2d(20, -50, Math.toRadians(-135)), 0)
                        .build();

        Trajectory forward =
                drive.trajectoryBuilder(toZone2.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(5)
                        .build();

        Trajectory toLaunchZone =
                drive.trajectoryBuilder((GlobalStorage.wobbleCount > 1) ? forward.end() : toZone1.end())
                        .lineToLinearHeading(new Pose2d(-5, -40, 0))
                        .build();

        Trajectory toParking =
                drive.trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, -40, 0))
                        .build();

        return Arrays.asList(
                new TrajectoryWrapper(toZone1),
                new TrajectoryWrapper(toPickupSecondWobble),
                new TrajectoryWrapper(-175),
                new TrajectoryWrapper(toZone2),
                new TrajectoryWrapper(forward),
                new TrajectoryWrapper(toLaunchZone),
                new TrajectoryWrapper(toParking));
    }

    private static List<TrajectoryWrapper> buildFour(AutoRobot robot, Pose2d startPose) {
        SampleMecanumDrive drive = robot.getDrive();

        Trajectory toZone1 =
                drive.trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(60, -38, Math.toRadians(90)), Math.toRadians(-70))
                        .build();

        Trajectory toPickupSecondWobble =
                drive.trajectoryBuilder(toZone1.end())
                        .addDisplacementMarker(() -> robot.prepareArmToPickupWobbleGoal())
                        .splineToLinearHeading(new Pose2d(-32, -51, 0), Math.toRadians(180))
                        .build();

        Trajectory toZone2 =
                drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                        .lineToLinearHeading(new Pose2d(40, -58, Math.toRadians(180)))
                        .build();

        Trajectory forward =
                drive.trajectoryBuilder(toZone2.end())
                        .addDisplacementMarker(() -> robot.prepareShooter())
                        .forward(20)
                        .build();

        Trajectory toLaunchZone =
                drive.trajectoryBuilder((GlobalStorage.wobbleCount > 1) ? forward.end() : toZone1.end())
                        .lineToLinearHeading(new Pose2d(-5, -40, 0))
                        .build();

        Trajectory toParking =
                drive.trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, -40, 0))
                        .build();

        return Arrays.asList(
                new TrajectoryWrapper(toZone1),
                new TrajectoryWrapper(toPickupSecondWobble),
                new TrajectoryWrapper(-175),
                new TrajectoryWrapper(toZone2),
                new TrajectoryWrapper(forward),
                new TrajectoryWrapper(toLaunchZone),
                new TrajectoryWrapper(toParking));
    }
}
