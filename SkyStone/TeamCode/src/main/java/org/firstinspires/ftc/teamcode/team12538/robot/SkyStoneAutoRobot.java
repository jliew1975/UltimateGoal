package org.firstinspires.ftc.teamcode.team12538.robot;

import org.firstinspires.ftc.teamcode.team12538.drive.AutoDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.NewMecanumDrive;

public class SkyStoneAutoRobot extends CommonRobotHardware implements Robot {
    /**
     * We can swap the drive logic with either
     * MecanumDrive or NewMecanumDrive implementation here
     */
    public AutoDrive mecanumDrive = new NewMecanumDrive();

    public void init() {
        super.init();
        mecanumDrive.init();
    }

    public void strafeLeft(MecanumDrive.StrafingDirection direction, double speed, double distance) {
        mecanumDrive.encoderStrafe(direction, speed, distance, 5d);
    }
}
