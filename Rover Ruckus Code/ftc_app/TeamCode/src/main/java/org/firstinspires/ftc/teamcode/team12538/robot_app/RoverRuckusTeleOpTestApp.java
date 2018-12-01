package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotTest;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
public class RoverRuckusTeleOpTestApp extends RoverRuckusAutoApp {

    private ElapsedTime runtime = new ElapsedTime();

    private Object lock = new Object();
    private volatile boolean busy = false;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            OpModeUtils.init(this);
            OpModeUtils.setDisableInitPos(true);

            final AutoRobotTest robot = new AutoRobotTest();
            robot.init();
            robot.init_imu();

            robot.getCollector().getOuttakeSlide().setPosition(1d);
            robot.getCollector().flipCollectorBox(0d);

            // detector = createDetector();
            // detector.enable();

            waitForStart();

            while (opModeIsActive()) {
                // robot.player1controls(gamepad1);

                if(gamepad1.a) {
                    robot.getSheetMetal().setPosition(0.75);
                } else if(gamepad1.b) {
                    robot.getSheetMetal().setPosition(0.5);
                } else if(gamepad1.x) {
                    robot.getSheetMetal().setPosition(0.2);
                }

                telemetry.addData("Phone Tilt", robot.getPhoneTilt().getPosition());
                telemetry.update();
            }
        } finally {
            if(detector != null) {
                detector.disable();
            }
        }
    }
}
