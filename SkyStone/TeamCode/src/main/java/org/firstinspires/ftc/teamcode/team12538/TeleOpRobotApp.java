package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.RobotApp;

@TeleOp(name="Robot Tele", group="Linear Opmode")
public class TeleOpRobotApp extends RobotApp {
    @Override
    public void performRobotOperation() throws InterruptedException {
        try {
            SkyStoneTeleOpRobot robot = new SkyStoneTeleOpRobot();
            robot.init();

            waitForStart();

            if(isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                robot.mecanumDrive.navigateWithGamepad(gamepad1);
                robot.mecanumDrive.printTelemetry();
                telemetry.update();
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
