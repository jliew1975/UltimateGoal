package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotTest;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
public class RoverRuckusTeleOpTestApp extends RoverRuckusAutoApp {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            OpModeUtils.init(this);

            AutoRobotTest robot = new AutoRobotTest();
            robot.init();
            robot.init_imu();

            // detector = createDetector();
            // detector.enable();

            waitForStart();

            while (opModeIsActive()) {
                robot.player1controls(gamepad1);

                robot.getRobotLatch().printTelemetry();
                telemetry.update();
            }
        } finally {
            if(detector != null) {
                detector.disable();
            }
        }
    }
}
