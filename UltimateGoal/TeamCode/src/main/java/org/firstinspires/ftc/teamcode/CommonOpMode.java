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

    protected double calculatePowerShotAngle(PowerShotPos pos) {
        switch(pos) {
            case One:
                return ShooterUtils.calculatePowerShotAngle(pShotPose1);
            case Two:
                return ShooterUtils.calculatePowerShotAngle(pShotPose2);
            default:
                return ShooterUtils.calculatePowerShotAngle(pShotPose3);
        }
    }
}
