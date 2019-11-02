package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneTeleOpRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
public class TeleOpTestRobotApp extends RobotApp {
    @Override
    public void performRobotOperation() throws InterruptedException {
        try {
            AutoGamepad autoGamepad = new AutoGamepad();

            SkyStoneAutoRobot robot = new SkyStoneAutoRobot();
            robot.init();

            waitForStart();

            if(isStopRequested()) {
                return;
            }

            double power = 0.1;

            while (opModeIsActive()) {
                if(gamepad1.a) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.Backward, power,3.0);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.y) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.Forward, power,3.0);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.x) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.StrafeLeft, power,3.0);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.b) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.StrafeRight, power,3.0);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                }

                // robot.mecanumDrive.printEncoderValue();
                // telemetry.update();
                // robot.foundationClaw.control(gamepad1);
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
