package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.util.AutonomousColor;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;

public class TargetConstant {
    public static Pose2d pShotPose1;
    public static Pose2d pShotPose2;
    public static Pose2d pShotPose3;

    public static Pose2d towerPose;

    static {
        if(OpModeUtils.getGlobalStore().autoColor == AutonomousColor.Red) {
            pShotPose1 = new Pose2d(74, 4);
            pShotPose2 = new Pose2d(74, -4);
            pShotPose3 = new Pose2d(74, -10);

            towerPose = new Pose2d(72, -38);
        } else {
            pShotPose1 = new Pose2d(74, -4);
            pShotPose2 = new Pose2d(74, 4);
            pShotPose3 = new Pose2d(74, 10);

            towerPose = new Pose2d(72, 38);
        }
    }
}
