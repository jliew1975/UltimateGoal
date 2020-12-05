package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.components.TargetConstant;
import org.firstinspires.ftc.teamcode.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.AutonomousColor;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.firstinspires.ftc.teamcode.util.ShooterUtils;

public abstract class CommonOpMode extends LinearOpMode {
    public enum PowerShotPos { One, Two, Three }

    protected Pose2d pShotPose1;
    protected Pose2d pShotPose2;
    protected Pose2d pShotPose3;

    protected Pose2d towerPose;

    public void initTargetPoseValues() {
        pShotPose1 = TargetConstant.pShotPose1;
        pShotPose2 = TargetConstant.pShotPose2;
        pShotPose3 = TargetConstant.pShotPose3;
        towerPose = TargetConstant.towerPose;
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
        switch(pos) {
            case One:
                return calculatePowerShotAngle(robot, pShotPose1);
            case Two:
                return calculatePowerShotAngle(robot, pShotPose2);
            default:
                return calculatePowerShotAngle(robot, pShotPose3);
        }
    }

    protected double calculatePowerShotAngle(AutoRobot robot, Pose2d targetPose) {
        Pose2d currPose = robot.getDrive().getPoseEstimate();

        double heading = currPose.getHeading();
        if(heading > Math.PI) {
            heading = heading - (2 * Math.PI);
        }

        double distance = Math.sqrt(
                Math.pow((targetPose.getY() - currPose.getY()), 2) +
                        Math.pow((targetPose.getX() - currPose.getX()), 2)
        );

        return Math.atan((targetPose.getY() - currPose.getY())/(targetPose.getX() - currPose.getX())) - heading - Math.atan(5.17 / distance);
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
