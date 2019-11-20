package org.firstinspires.ftc.teamcode.team12538.utils;

import org.firstinspires.ftc.teamcode.team12538.components.RobotOuttakeSlides;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;

public class RobotUtils {
    public static void deploySkyStone(SkyStoneAutoRobot robot) {
        robot.outtake.prepareForStoneDeployment();

        ThreadUtils.sleep(500);

        robot.outtake.outtakeSlides.runToPosition(RobotOuttakeSlides.ENCODER_TICKS_FOR_STONE_DROP);
        robot.outtake.performOuttakeClawOperation();

        ThreadUtils.sleep(500);
    }

    public static void prepareForStonePickup(SkyStoneAutoRobot robot) {
        robot.intake.setPower(0d);
        robot.outtake.lowerSlideForStonePickup();
    }
}
