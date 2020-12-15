package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class GlobalStorage {
    public static int wobbleCount = 1;
    public static AutoConstant.Side side = AutoConstant.Side.Left;
    public static AutonomousColor autoColor = AutonomousColor.Red;

    public static Pose2d currentPose = new Pose2d();
}
