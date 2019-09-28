package org.firstinspires.ftc.teamcode.team12538;

import org.firstinspires.ftc.teamcode.team12538.components.AutoDrive;
import org.firstinspires.ftc.teamcode.team12538.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.components.NewMecanumDrive;

public class SkyStoneAutoRobot extends CommonRobotHardware implements Robot  {
    /**
     * We can swap the drive logic with either
     * MecanumDrive or NewMecanumDrive implementation here
     */
    protected AutoDrive mecanumDrive = new NewMecanumDrive();

    public void strafeLeft(MecanumDrive.StrafingDirection direction, double speed, double distance) {
        mecanumDrive.encoderStrafe(direction, speed, distance, 5d);
    }
}
