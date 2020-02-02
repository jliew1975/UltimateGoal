package org.firstinspires.ftc.teamcode.team12538.robot;

import org.firstinspires.ftc.teamcode.team12538.drive.AutoDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.PIDMecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class SkyStoneAutoRobot extends CommonRobotHardware implements Robot {
    /**
     * We can swap the drive logic with either
     * MecanumDrive or NewMecanumDrive implementation here
     */
    public AutoDrive mecanumDrive = new NewMecanumDrive();

    public SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(OpModeUtils.getHardwareMap());

    public void init() {
        super.init();
        // mecanumDrive.init();
        // mecanumDrive.init_imu();
    }
}
