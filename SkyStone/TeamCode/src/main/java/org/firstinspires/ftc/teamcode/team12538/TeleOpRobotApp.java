package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneTeleOpRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

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
                // mecanum drive controls
                robot.getMecanumDrive().navigateWithGamepad(gamepad1);
                if(gamepad1.a) {
                    robot.getMecanumDrive().resetEncoderValues();
                }

                // intake controls
                if(gamepad1.right_bumper) {
                    robot.getIntake().setPower(1d);
                } else if(gamepad1.left_bumper) {
                    robot.getIntake().setPower(-1d);
                } else {
                    robot.getIntake().setPower(0d);
                }

                robot.getOuttakeSlides().control(gamepad1);

                // telemetry printing
                robot.getMecanumDrive().printTelemetry();
                robot.getOuttakeSlides().printTelemetry();
                telemetry.update();
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
