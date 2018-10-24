package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.components.MineralCollector;
import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotTest;
import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotV1;
import org.firstinspires.ftc.teamcode.team12538.robotV1.TeleOpRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
public class RoverRuckusTeleOpTestApp extends RoverRuckusAutoApp {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            OpModeUtils.getGlobalStore().setOpMode(this);
            OpModeUtils.getGlobalStore().setHardwareMap(hardwareMap);
            OpModeUtils.getGlobalStore().setTelemetry(telemetry);

            AutoRobotTest robot = new AutoRobotTest();
            robot.init();
            robot.init_imu();

            detector = createDetector();
            detector.enable();

            telemetry.addData("Angle", robot.getAngle());
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                robot.controlWithGamePad(gamepad1);

                // mineral intake mechanis
                robot.controlMineralArm(gamepad2.left_stick_x);

                if (gamepad1.a) {
                    robot.rotate(23, 0.01);
                    robot.stop();
                } else if (gamepad1.b) {
                    robot.rotate(-23, 0.01);
                    robot.stop();
                }

                if(gamepad1.x) {
                    robot.moveBackward(0.01);
                    while (gamepad1.x) {
                        idle();
                    }

                    robot.stop();
                }

                if(gamepad1.y) {
                    robot.moveForward(0.5, 10);
                    /*
                    robot.moveForward(0.01);
                    while (gamepad1.y) {
                        idle();
                    }

                    robot.stop();
                    */
                }

                robot.printDriveEncoderTelemtry();
                telemetry.addData("Gold Mineral Found", detector.isFound());
                telemetry.addData( "Gold Mineral X-Pos", detector.getXPosition());
                telemetry.update();
            }
        } finally {
            if(detector != null) {
                detector.disable();
            }
        }
    }
}
