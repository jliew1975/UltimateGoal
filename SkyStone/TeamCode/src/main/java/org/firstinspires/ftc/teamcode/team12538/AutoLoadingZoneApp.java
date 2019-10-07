package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team12538.utils.RobotApp;

@Autonomous(name="Auto (Loading Zone)", group="Linear Opmode")
public class AutoLoadingZoneApp extends RobotApp {
    @Override
    public void performRobotOperation() throws InterruptedException {
        SkyStoneAutoRobot robot = new SkyStoneAutoRobot();
        robot.init();

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                robot.mecanumDrive.encoderDrive(0.1, 1.0, 10.0);
            }
        }
    }
}
