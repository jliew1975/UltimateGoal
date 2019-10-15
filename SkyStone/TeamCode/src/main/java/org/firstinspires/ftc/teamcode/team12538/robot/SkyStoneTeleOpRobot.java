package org.firstinspires.ftc.teamcode.team12538.robot;

import org.firstinspires.ftc.teamcode.team12538.components.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.components.TeleOpDrive;

public class SkyStoneTeleOpRobot extends CommonRobotHardware implements Robot {
    // Declare TeleOp specific hardware component here

    /**
     * We can swap the drive logic with either
     * MecanumDrive or NewMecanumDrive implementation here
     */
    protected TeleOpDrive mecanumDrive = new NewMecanumDrive();

    @Override
    public void init() {
        super.init();
        mecanumDrive.init();
    }

    public TeleOpDrive getMecanumDrive() {
        return mecanumDrive;
    }
}
