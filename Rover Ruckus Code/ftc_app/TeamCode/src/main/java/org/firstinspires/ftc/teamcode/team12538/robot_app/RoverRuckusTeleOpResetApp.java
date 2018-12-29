package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.TeleOpRobotReset;
import org.firstinspires.ftc.teamcode.team12538.robotV1.TeleOpRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele v1 (Reset)", group="Linear Opmode")
@Disabled
public class RoverRuckusTeleOpResetApp extends RoverRuckusTeleOpApp {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        OpModeUtils.getGlobalStore().setCloseDepoArm(false);
        OpModeUtils.getGlobalStore().setDisableInitPos(true);
        OpModeUtils.getGlobalStore().setResetArmExtensionEncoderValue(true);

        try {
            TeleOpRobotV1 robot = new TeleOpRobotV1();
            robot.init();

            waitForStart();

            // tilt the phone for mineral scanning
            robot.getPhoneTilt().setPosition(1d);

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
