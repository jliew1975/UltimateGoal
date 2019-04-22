package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.TeleOpRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele", group="Linear Opmode")
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

            waitForStart();

            if(isStopRequested()) {
                return;
            }

            // tilt the phone for teleOp mode
            robot.getParkingRod().setPosition(1d);
            robot.getCameraTilt().setPosition(0d);

            while (opModeIsActive()) {
                robot.player1controls(gamepad1);
                robot.player2Controls(gamepad2);

                // robot.getCollector().printTelemetry();
                // robot.getRobotLatch().printTelemetry();
                // robot.printDeadWheelTelemetry();
                telemetry.addData("rightTrigger", gamepad2.right_trigger);
                telemetry.update();
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
