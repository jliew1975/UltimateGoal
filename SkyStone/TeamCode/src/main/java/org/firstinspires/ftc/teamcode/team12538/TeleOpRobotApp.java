package org.firstinspires.ftc.teamcode.team12538;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneArm;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneTeleOpRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele", group="Linear Opmode")
public class TeleOpRobotApp extends RobotApp {
    @Override
    public void performRobotOperation() throws InterruptedException {
        try {
            // Tell global store that runMode is in TeleOp mode
            OpModeUtils.getGlobalStore().runMode = OpModeStore.RunMode.TeleOp;

            SkyStoneTeleOpRobot robot = new SkyStoneTeleOpRobot();
            robot.init();

            waitForStart();

            if(isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                // mecanum drive controls
                robot.mecanumDrive.navigateWithGamepad(gamepad1);

                robot.intake.control(gamepad1);
                robot.outtake.control(gamepad1);
                robot.foundationClaw.control(gamepad2);
                robot.outtake.conttrolForCapstone(gamepad2);

                if(gamepad2.x) {
                    robot.stoneArm.setPosition(RobotStoneArm.UP);
                } else if(gamepad2.b) {
                    robot.stoneArm.setPosition(RobotStoneArm.DOWN);
                }


                // telemetry printing
                // robot.mecanumDrive.printTelemetry();
                // robot.outtake.printTelemetry();
                // robot.intake.printTelemetry();
                // telemetry.update();
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
