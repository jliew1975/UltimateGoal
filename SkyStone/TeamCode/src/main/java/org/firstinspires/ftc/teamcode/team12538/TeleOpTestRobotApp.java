package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneTeleOpRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
public class TeleOpTestRobotApp extends RobotApp {
    @Override
    public void performRobotOperation() throws InterruptedException {
        try {
            SkyStoneAutoRobot robot = new SkyStoneAutoRobot();
            robot.init();

            waitForStart();

            if(isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                if(gamepad1.a) {
                    robot.getMecanumDrive().encoderDrive(0.1, 5.0, 5.0);
                }
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
