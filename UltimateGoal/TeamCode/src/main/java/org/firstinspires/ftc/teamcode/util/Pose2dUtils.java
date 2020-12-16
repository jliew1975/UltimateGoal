package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Pose2dUtils {
    public static Pose2d updateHeading(Pose2d inputPose, double heading) {
        return new Pose2d(inputPose.getX(), inputPose.getY(), heading);
    }
}
