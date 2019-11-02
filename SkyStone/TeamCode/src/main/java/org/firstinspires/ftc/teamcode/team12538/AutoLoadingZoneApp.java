package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive.AutoDirection;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@Autonomous(name="Auto (Loading Zone)", group="Linear Opmode")
public class AutoLoadingZoneApp extends RobotApp {

    @Override
    public void performRobotOperation() throws InterruptedException {
        // Enable encoder
        OpModeUtils.setDriveEncoderEnabled(true);

        // Initialize a autonomous gamepad
        AutoGamepad gamepad = new AutoGamepad();

        SkyStoneAutoRobot robot = new SkyStoneAutoRobot();
        robot.init();

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                AutoGamepadUtils.move(gamepad, AutoDirection.Forward,0.1,3.0);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
            }
        }
    }
}
