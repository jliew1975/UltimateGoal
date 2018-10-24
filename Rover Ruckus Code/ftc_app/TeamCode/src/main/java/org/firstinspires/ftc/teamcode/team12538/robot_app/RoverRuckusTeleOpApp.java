package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.components.MineralCollector;
import org.firstinspires.ftc.teamcode.team12538.robotV1.TeleOpRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele v1", group="Linear Opmode")
public class RoverRuckusTeleOpApp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.getGlobalStore().setHardwareMap(hardwareMap);
        OpModeUtils.getGlobalStore().setTelemetry(telemetry);

        TeleOpRobotV1 robot = new TeleOpRobotV1();
        robot.init();

        waitForStart();

        while(opModeIsActive()) {
            robot.controlWithGamePad(gamepad1);

            // mineral intake mechanism
            robot.controlMineralArm(gamepad2.left_stick_x);

            if(gamepad2.y) {
                robot.liftArm();
            } else if(gamepad2.b) {
                robot.lowerArm();
            }

            if(gamepad2.dpad_down) {
                robot.lowerArm(0.01);
            } else if(gamepad2.dpad_up) {
                robot.liftArm(0.01);
            }

            if(gamepad2.a) {
                robot.turnOnIntake(MineralCollector.Direction.Forward);
            } else if(gamepad2.x) {
                robot.turnOnIntake(MineralCollector.Direction.Backward);
            } else {
                robot.turnOffIntake();
            }
        }
    }
}
