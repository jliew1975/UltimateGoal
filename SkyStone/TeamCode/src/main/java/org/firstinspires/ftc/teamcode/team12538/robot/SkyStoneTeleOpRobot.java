package org.firstinspires.ftc.teamcode.team12538.robot;

import org.firstinspires.ftc.teamcode.team12538.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.TeleOpDrive;

public class SkyStoneTeleOpRobot extends CommonRobotHardware implements Robot {
    // Declare TeleOp specific hardware component here

    /**
     * We can swap the drive logic with either
     * MecanumDrive or NewMecanumDrive implementation here
     */
    public TeleOpDrive mecanumDrive = new NewMecanumDrive();

    @Override
    public void init() {
        super.init();
        mecanumDrive.init();
    }
}
