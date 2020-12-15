package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.detectors.StarterRingsDetector.RingCount;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryWrapper;
import org.firstinspires.ftc.teamcode.util.AutoConstant.Side;
import org.firstinspires.ftc.teamcode.util.AutonomousColor;
import org.firstinspires.ftc.teamcode.util.GlobalStorage;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

import java.util.List;

@Autonomous(name="Red (L)", group="Group 1")
public class AutoRedLeftApp extends AutoApp {
    public AutoRedLeftApp() {
        GlobalStorage.side = Side.Left;
        GlobalStorage.autoColor = AutonomousColor.Red;
        GlobalStorage.wobbleCount = 1;
    }

    @Override
    public void performRobotOperation() {
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
        List<TrajectoryWrapper> trajectories = trajectoryFactory.getTrajectories(RingCount.NONE);

        SampleMecanumDrive drive = robot.getDrive();

        int driveIndex = 0;
        drive.followTrajectory(trajectories.get(driveIndex++).get()); // toZoneA
        robot.dropWobbleGoal(false);

        if(GlobalStorage.wobbleCount > 1) {
            drive.followTrajectory(trajectories.get(driveIndex++).get()); // toPickupWobble
            robot.pickupWobbleGoal();
            drive.turn(trajectories.get(driveIndex++).get()); // turn
            drive.followTrajectory(trajectories.get(driveIndex++).get()); // toZoneA
            robot.dropWobbleGoal();
            drive.followTrajectory(trajectories.get(driveIndex++).get()); // forward
        }

        drive.followTrajectory(trajectories.get(driveIndex++).get()); // toLaunchZone
        shootPowerShot();
        drive.followTrajectory(trajectories.get(driveIndex++).get()); // toParking
    }


    private void performOne() {
        List<TrajectoryWrapper> trajectories = trajectoryFactory.getTrajectories(RingCount.NONE);

        SampleMecanumDrive drive = robot.getDrive();

        int driveIndex = 0;
        drive.followTrajectory(trajectories.get(driveIndex++).get()); // toZoneB
        robot.dropWobbleGoal(false);

        if(GlobalStorage.wobbleCount > 1) {
            drive.followTrajectory(trajectories.get(driveIndex++).get()); // toPickupWobble
            robot.pickupWobbleGoal();
            drive.turn(trajectories.get(2).get()); // turn
            drive.followTrajectory(trajectories.get(driveIndex++).get()); // toZoneB
            robot.dropWobbleGoal();
            drive.followTrajectory(trajectories.get(driveIndex++).get()); // forward
        }

        drive.followTrajectory(trajectories.get(driveIndex++).get()); // toLaunchZone
        shootPowerShot();
        drive.followTrajectory(trajectories.get(driveIndex++).get()); // toParking
    }

    private void performFour() {
        List<TrajectoryWrapper> trajectories = trajectoryFactory.getTrajectories(RingCount.NONE);

        SampleMecanumDrive drive = robot.getDrive();

        int driveIndex = 0;
        drive.followTrajectory(trajectories.get(driveIndex++).get()); // toZoneC
        robot.dropWobbleGoal(false);

        if(GlobalStorage.wobbleCount > 1) {
            drive.followTrajectory(trajectories.get(driveIndex++).get()); // toPickupWobble
            robot.pickupWobbleGoal();
            drive.turn(trajectories.get(driveIndex++).get()); // turn
            drive.followTrajectory(trajectories.get(driveIndex++).get()); // toZoneC
            robot.dropWobbleGoal();
            drive.followTrajectory(trajectories.get(driveIndex++).get()); // forward
        }

        drive.followTrajectory(trajectories.get(driveIndex++).get()); // toLaunchZone
        shootPowerShot();
        drive.followTrajectory(trajectories.get(driveIndex++).get()); // toParking
    }
}
