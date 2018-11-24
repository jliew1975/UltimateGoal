package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.TeleOpRobotReset;
import org.firstinspires.ftc.teamcode.team12538.robotV1.TeleOpRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele (Reset)", group="Linear Opmode")
public class RoverRuckusTeleOpResetApp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        OpModeUtils.getGlobalStore().setDisableLimit(true);

        try {
            TeleOpRobotReset robot = new TeleOpRobotReset();
            robot.init();
            robot.getSheetMetal().setPosition(1.0);

            waitForStart();

            // tilt the phone for mineral scanning
            robot.getPhoneTilt().setPosition(0.23);

            while (opModeIsActive()) {
                robot.player1controls(gamepad1);
                robot.player2Controls(gamepad2);

                robot.getCollector().printTelemetry();
                robot.getRobotLatch().printTelemetry();
                telemetry.update();
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
