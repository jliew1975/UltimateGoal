package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotTest;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
@Disabled
public class RoverRuckusTeleOpTestApp extends RoverRuckusAutoApp {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            OpModeUtils.init(this);

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


                if (gamepad1.a) {
                    robot.rotate(23, 0.3);
                    robot.stop();
                } else if (gamepad1.b) {
                    robot.rotate(-23, 0.3);
                    robot.stop();
                }

                robot.printImuAngleTelemtry();

                /*
                // mineral intake mechanis
                robot.getCollector().controlArm(-gamepad1.left_stick_x);

                if(gamepad1.x) {
                    robot.getCollector().adjustArmPosition(-100, false);
                }

                if(gamepad1.a) {
                    robot.getCollector().flipCollectorBox(0.6);
                }
                */
            }
        } finally {
            if(detector != null) {
                detector.disable();
            }
        }
    }
}
