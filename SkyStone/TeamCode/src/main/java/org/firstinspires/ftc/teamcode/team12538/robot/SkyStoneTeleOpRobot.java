package org.firstinspires.ftc.teamcode.team12538.robot;

import org.firstinspires.ftc.teamcode.team12538.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.PIDMecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.TeleOpDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class SkyStoneTeleOpRobot extends CommonRobotHardware implements Robot {
    // Declare TeleOp specific hardware component here

    /**
     * We can swap the drive logic with either
     * MecanumDrive or NewMecanumDrive implementation here
     */
    public TeleOpDrive mecanumDrive = new PIDMecanumDrive();

    /*
     * For Auto function in teleop.
     */
    public SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(OpModeUtils.getHardwareMap());

    @Override
    public void init() {
        super.init();
        mecanumDrive.init();
    }
}
