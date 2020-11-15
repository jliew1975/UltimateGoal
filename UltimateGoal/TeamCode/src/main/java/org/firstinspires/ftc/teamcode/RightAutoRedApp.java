package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.detectors.StarterRingsDetector.RingCount;
import org.firstinspires.ftc.teamcode.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.AutonomousColor;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@Autonomous(name="Red (R)", group="Group 1")
public class RightAutoRedApp extends AutoApp {
    private Pose2d startPose = new Pose2d(-62, -50, Math.toRadians(180));

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
        Trajectory toZoneA =
                robot.getDrive().trajectoryBuilder(startPose, true)
                        .splineTo(new Vector2d(-5, -60), 180)
                        .build();

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(toZoneA.end())
                        .lineToSplineHeading(new Pose2d(-5, -15, 0))
                        .build();

        robot.getDrive().turn(Math.toRadians(-90));

        Trajectory toParking =
                robot.getDrive().trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, -10, 0))
                        .build();

        robot.getDrive().followTrajectory(toZoneA);
        robot.dropWobbleGoal();
        robot.getDrive().followTrajectory(toLaunchZone);
        shootPowerShot();
        robot.getDrive().followTrajectory(toParking);
    }


    private void performOne() {
        Trajectory toZoneB =
                robot.getDrive().trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(23, -48, Math.toRadians(35)), Math.toRadians(-145))
                        .addDisplacementMarker(() -> robot.prepareWobbleArm() )
                        .build();

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(toZoneB.end())
                        .lineToSplineHeading(new Pose2d(-5, -15, 0))
                        .build();

        Trajectory toParking =
                robot.getDrive().trajectoryBuilder(toLaunchZone.end())
                        .lineToLinearHeading(new Pose2d(10, -10, 0))
                        .build();

        robot.getDrive().followTrajectory(toZoneB);
        robot.dropWobbleGoal();
        // robot.getDrive().followTrajectory(toLaunchZone);
        // shootPowerShot();
        // robot.getDrive().followTrajectory(toParking);
    }

    private void performFour() {
        Trajectory toZoneC =
                robot.getDrive().trajectoryBuilder(startPose, true)
                        .splineToSplineHeading(new Pose2d(45, -60, Math.toRadians(180)), 0)
                        .addDisplacementMarker(() -> robot.prepareWobbleArm() )
                        .build();

        Trajectory toLaunchZone =
                robot.getDrive().trajectoryBuilder(toZoneC.end())
                        .lineToLinearHeading(new Pose2d(-5, -15, 0))
                        .build();

        Trajectory toParking =
                robot.getDrive().trajectoryBuilder(toLaunchZone.end())
                        .splineToSplineHeading(new Pose2d(10, -10, Math.toRadians(0)), Math.toRadians(0))
                        .build();

        robot.getDrive().followTrajectory(toZoneC);
        robot.dropWobbleGoal();
        // robot.getDrive().followTrajectory(toLaunchZone);
        // shootPowerShot();
        // robot.getDrive().followTrajectory(toParking);
    }

    private void shootPowerShot() {
        Shooter shooter = robot.get(Shooter.class);
        shooter.liftShooter(2);
        shooter.start();
        ThreadUtils.sleep(1000);
        try {
            robot.getDrive().turn(calculateTargetAngle(robot, 1));
            shooter.fire();
            ThreadUtils.sleep(1000);
            robot.getDrive().turn(calculateTargetAngle(robot, 2));
            shooter.fire();
            ThreadUtils.sleep(1000);
            robot.getDrive().turn(calculateTargetAngle(robot, 3));
            shooter.fire();
            sleep(2000);
        } finally {
            shooter.stop();
        }
    }

    private double calculateTargetAngle(AutoRobot robot, int powerShotPos) {
        double x1 = 74, y1 = 6;
        double x2 = 74, y2 = -2;
        double x3 = 74, y3 = -8;

        Pose2d currPose = robot.getDrive().getPoseEstimate();

        double heading = currPose.getHeading();
        if(heading > Math.PI) {
            heading = heading - (2 * Math.PI);
        }

        if(powerShotPos == 1) {
            return Math.atan((y1 - currPose.getY())/(x1 - currPose.getX()) - heading);
        } else if(powerShotPos == 2) {
            return Math.atan((y2 - currPose.getY())/(x2 - currPose.getX()) - heading);
        } else {
            return Math.atan((y3 - currPose.getY())/(x3 - currPose.getX()) - heading);
        }
    }
}
