package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.AutonomousColor;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;

public abstract class CommonOpMode extends LinearOpMode {
    public enum PowerShotPos { One, Two, Three }

    protected Pose2d pShotPose1;
    protected Pose2d pShotPose2;
    protected Pose2d pShotPose3;

    protected Pose2d towerPose;

    public void initTargetPoseValues() {
        if(OpModeUtils.getGlobalStore().autoColor == AutonomousColor.Red) {
            pShotPose1 = new Pose2d(74, 4);
            pShotPose2 = new Pose2d(74, -4);
            pShotPose3 = new Pose2d(74, -10);

            towerPose = new Pose2d(72, -36);
        } else {
            pShotPose1 = new Pose2d(74, -4);
            pShotPose2 = new Pose2d(74, 4);
            pShotPose3 = new Pose2d(74, 10);

            towerPose = new Pose2d(72, 36);
        }
    }

    protected double calculateTowerAngle(AutoRobot robot) {
        Pose2d currPose = robot.getDrive().getPoseEstimate();

        double heading = currPose.getHeading();
        if(heading > Math.PI) {
            heading = heading - (2 * Math.PI);
        }

        return Math.atan((towerPose.getY() - currPose.getY())/(towerPose.getX() - currPose.getX()) - heading);
    }

    protected double calculatePowerShotAngle(AutoRobot robot, PowerShotPos pos) {
        Pose2d currPose = robot.getDrive().getPoseEstimate();

        double heading = currPose.getHeading();
        if(heading > Math.PI) {
            heading = heading - (2 * Math.PI);
        }

        switch(pos) {
            case One:
                return Math.atan((pShotPose1.getY() - currPose.getY())/(pShotPose1.getX() - currPose.getX()) - heading);
            case Two:
                return Math.atan((pShotPose2.getY() - currPose.getY())/(pShotPose2.getX() - currPose.getX()) - heading);
            default:
                return Math.atan((pShotPose3.getY() - currPose.getY())/(pShotPose3.getX() - currPose.getX()) - heading);
        }
    }

    protected double calculateShooterSpeed(AutoRobot robot) {
        Pose2d currPose = robot.getDrive().getPoseEstimate();
        double hDist = toMeter(
                Math.sqrt(
                        Math.pow(towerPose.getX() - currPose.getX(), 2) +
                                Math.pow(towerPose.getY() - currPose.getY(), 2)));

        double vDist = toMeter(35);
        double yVelocity = Math.sqrt(19.6 * vDist);
        double xVelocity = hDist/(Math.sqrt(vDist/4.9));

        return (Math.sqrt(Math.pow(yVelocity, 2) + Math.pow(xVelocity,2)) * 2)/0.010668;
    }

    protected double toMeter(double inches) {
        return inches/39.37;
    }
}
