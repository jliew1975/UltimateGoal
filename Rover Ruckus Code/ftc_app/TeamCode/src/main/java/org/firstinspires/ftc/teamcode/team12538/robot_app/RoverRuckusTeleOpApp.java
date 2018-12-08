package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.TeleOpRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@TeleOp(name="Robot Tele v1", group="Linear Opmode")
public class RoverRuckusTeleOpApp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        OpModeUtils.getGlobalStore().setCloseDepoArm(false);
        OpModeUtils.getGlobalStore().setDisableInitPos(true);
        OpModeUtils.getGlobalStore().setResetArmExtensionEncoderValue(false);

        try {
            TeleOpRobotV1 robot = new TeleOpRobotV1();
            robot.init();
            robot.getSheetMetal().setPosition(0.75);
            robot.getCollector().getOuttakeSlide().setPosition(1d);

            waitForStart();

            // tilt the phone for mineral scanning
            robot.getPhoneTilt().setPosition(0.4);

            while (opModeIsActive()) {
                robot.player1controls(gamepad1);
                robot.player2Controls(gamepad2);

                robot.getCollector().printTelemetry();
                robot.getRobotLatch().printTelemetry();
                telemetry.addData("gamepad2.left_stick_x", gamepad2.left_stick_x);
                telemetry.update();
            }

        } finally {
            OpModeUtils.stop();
        }
    }
}
