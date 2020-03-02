package org.firstinspires.ftc.teamcode.team12538.robot;

import org.firstinspires.ftc.teamcode.team12538.drive.AutoDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.PIDMecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.team12538.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class SkyStoneAutoRobot extends CommonRobotHardware implements Robot {
    /**
     * Old Mecanum drive logic
     */
    public AutoDrive mecanumDrive = null;

    /**
     * Road Runner drive logic
     */
    public SampleMecanumDriveBase drive = null;

    public void init() {
        drive = new SampleMecanumDriveREVOptimized(OpModeUtils.getHardwareMap());
        super.init();
    }

    public void init_old_drive() {
        super.init();
        mecanumDrive = new NewMecanumDrive();
        mecanumDrive.init();
        mecanumDrive.init_imu();
    }
}
